// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The mesh corpus: generator, and the check that the validator classifies every
//! entry the way it is supposed to.
//!
//! # How this is kept honest
//!
//! The mesh files are **generated** by [`regenerate_corpus`], so they can be
//! reproduced and diffed rather than being opaque blobs somebody once exported.
//! They are also **committed**, so the tests are fast and do not depend on the
//! generator still working.
//!
//! The expectations live in `tests/corpus/mesh/expectations.json`, which is
//! **hand-written**. That matters: an expectation file generated from the
//! validator's own output would assert only that the validator is
//! self-consistent, which is worth nothing. Each entry says what the mesh was
//! built to exhibit, and the test asserts the validator agrees.
//!
//! [`every_corpus_entry_is_classified_correctly`] is the exit criterion for
//! Unit 2's validation work.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use chipbreaker_core::eps::EPS_WELD;
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::io::{self, Format};
use chipbreaker_core::mesh::units::Unit;
use chipbreaker_core::mesh::validate::{check_self_intersections, validate};
use chipbreaker_core::mesh::weld::weld;
use chipbreaker_core::mesh::{MeshMeta, TriMesh, shapes};
use serde_json::Value;

fn corpus_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/corpus/mesh")
}

fn mesh_of(v: Vec<Vec3>, t: Vec<[u32; 3]>) -> TriMesh {
    TriMesh::new(v, t, MeshMeta::synthetic()).expect("generator produced a valid mesh")
}

/// Offsets a mesh's indices and appends it to another, giving a second component.
fn combine(a: &TriMesh, b: &TriMesh) -> TriMesh {
    let mut v = a.vertices().to_vec();
    let mut t = a.triangles().to_vec();
    let offset = v.len() as u32;
    v.extend_from_slice(b.vertices());
    t.extend(
        b.triangles()
            .iter()
            .map(|x| [x[0] + offset, x[1] + offset, x[2] + offset]),
    );
    mesh_of(v, t)
}

/// The well-formed solids: analytic shapes at three tessellations each, plus the
/// lattice-aligned adversarial blocks.
fn good_meshes() -> Vec<(&'static str, TriMesh)> {
    vec![
        ("cube-coarse", shapes::cube(10.0)),
        (
            "box-oblong",
            shapes::box_solid(Vec3::new(-3.0, -5.0, -7.0), Vec3::new(4.0, 6.0, 8.0)),
        ),
        ("sphere-0", shapes::icosphere(5.0, 0)),
        ("sphere-1", shapes::icosphere(5.0, 1)),
        ("sphere-2", shapes::icosphere(5.0, 2)),
        ("cylinder-8", shapes::cylinder(4.0, 9.0, 8)),
        ("cylinder-32", shapes::cylinder(4.0, 9.0, 32)),
        ("cylinder-128", shapes::cylinder(4.0, 9.0, 128)),
        ("cone-8", shapes::cone(4.0, 9.0, 8)),
        ("cone-32", shapes::cone(4.0, 9.0, 32)),
        ("cone-128", shapes::cone(4.0, 9.0, 128)),
        ("torus-16", shapes::torus(6.0, 2.0, 16, 8)),
        ("torus-32", shapes::torus(6.0, 2.0, 32, 16)),
        ("torus-64", shapes::torus(6.0, 2.0, 64, 32)),
        ("lattice-1", shapes::lattice_block(1)),
        ("lattice-3", shapes::lattice_block(3)),
        ("lattice-5", shapes::lattice_block(5)),
    ]
}

/// The deliberately broken meshes that still load as valid geometry.
///
/// Each is built to exhibit exactly one defect class, in isolation, so the
/// expectation is unambiguous. Piling a defect onto a cube tends to create
/// incidental extra findings — a spurious triangle on a cube edge makes that
/// edge non-manifold *and* opens a boundary — which makes the expectation a
/// puzzle rather than a statement.
fn broken_meshes() -> Vec<(&'static str, TriMesh)> {
    let cube = shapes::cube(10.0);

    // Three triangles sharing one edge: non-manifold, with six boundary edges
    // around the outside of the fan.
    let non_manifold = mesh_of(
        vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(0.0, 10.0, 0.0),
            Vec3::new(0.0, -10.0, 0.0),
            Vec3::new(0.0, 0.0, 10.0),
        ],
        vec![[0, 1, 2], [0, 1, 3], [0, 1, 4]],
    );

    // A cube with one facet removed: a triangular hole.
    let mut open = cube.triangles().to_vec();
    open.remove(0);
    let open = mesh_of(cube.vertices().to_vec(), open);

    // A cube with a single facet wound backwards.
    let mut flipped = cube.triangles().to_vec();
    flipped[0] = [flipped[0][0], flipped[0][2], flipped[0][1]];
    let flipped = mesh_of(cube.vertices().to_vec(), flipped);

    // Every facet wound backwards: locally consistent, globally inside out.
    let inverted = mesh_of(
        cube.vertices().to_vec(),
        cube.triangles()
            .iter()
            .map(|t| [t[0], t[2], t[1]])
            .collect(),
    );

    // A closed cube plus an isolated triangle with a repeated index. The
    // repeated index makes it degenerate; being isolated keeps the cube's
    // topology untouched.
    let mut zero_v = cube.vertices().to_vec();
    let mut zero_t = cube.triangles().to_vec();
    let base = zero_v.len() as u32;
    zero_v.extend_from_slice(&[Vec3::splat(50.0), Vec3::splat(51.0), Vec3::splat(52.0)]);
    zero_t.push([base, base + 1, base + 1]);
    let zero_area = mesh_of(zero_v, zero_t);

    // A cube plus an isolated triangle whose three vertices are exactly
    // collinear: degenerate, and its three edges are boundaries.
    let mut collinear_v = cube.vertices().to_vec();
    let mut collinear_t = cube.triangles().to_vec();
    let base = collinear_v.len() as u32;
    collinear_v.extend_from_slice(&[
        Vec3::new(50.0, 0.0, 0.0),
        Vec3::new(51.0, 0.0, 0.0),
        Vec3::new(53.0, 0.0, 0.0),
    ]);
    collinear_t.push([base, base + 1, base + 2]);
    let collinear = mesh_of(collinear_v, collinear_t);

    // Two coincident triangles with opposite windings. Every edge still has
    // exactly two consistently wound uses, so it is manifold and watertight —
    // and encloses nothing.
    let duplicate = mesh_of(
        vec![
            Vec3::ZERO,
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(0.0, 10.0, 0.0),
        ],
        vec![[0, 1, 2], [0, 2, 1]],
    );

    // Two cubes overlapping in space.
    let self_intersecting = combine(
        &cube,
        &shapes::box_solid(Vec3::splat(5.0), Vec3::splat(15.0)),
    );

    // Two cubes far apart.
    let disjoint = combine(
        &cube,
        &shapes::box_solid(Vec3::splat(50.0), Vec3::splat(60.0)),
    );

    // A small sphere entirely inside a large one. Both outward-oriented, so the
    // volumes add rather than subtract — the question U5 will care about.
    let nested = combine(&shapes::icosphere(10.0, 2), &shapes::icosphere(3.0, 2));

    vec![
        ("broken-nonmanifold", non_manifold),
        ("broken-open", open),
        ("broken-flipped-face", flipped),
        ("broken-inverted", inverted),
        ("broken-zero-area", zero_area),
        ("broken-collinear", collinear),
        ("broken-duplicate-tri", duplicate),
        ("broken-selfintersect", self_intersecting),
        ("broken-two-components", disjoint),
        ("broken-nested", nested),
    ]
}

/// Malformed *files*, which must be rejected or tolerated at the parser rather
/// than reaching the validator.
fn malformed_files() -> Vec<(&'static str, Vec<u8>)> {
    let cube = shapes::cube(10.0);
    let good = io::stl::write_binary(&cube);

    // A NaN coordinate, planted in the first vertex of the first facet.
    let mut nan = good.clone();
    nan[96..100].copy_from_slice(&f32::NAN.to_le_bytes()); // ALLOW-f32-WIRE-FORMAT

    // An infinite coordinate, likewise.
    let mut infinite = good.clone();
    infinite[96..100].copy_from_slice(&f32::INFINITY.to_le_bytes()); // ALLOW-f32-WIRE-FORMAT

    // Truncated part-way through the triangle array.
    let mut truncated = good.clone();
    truncated.truncate(good.len() - 37);

    // A header that claims more triangles than the file holds.
    let mut wrong_count = good.clone();
    wrong_count[80..84].copy_from_slice(&999u32.to_le_bytes());

    vec![
        ("broken-nan", nan),
        ("broken-inf", infinite),
        ("broken-truncated", truncated),
        ("broken-wrong-count", wrong_count),
        ("broken-empty", Vec::new()),
    ]
}

/// Text-format entries: OBJ quirks and a tolerated ASCII STL malformation.
fn text_files() -> Vec<(&'static str, &'static str)> {
    vec![
        // A coordinate outside the range where orient3d is exact. It cannot be
        // expressed through binary STL, whose f32 tops out at 3.4e38, so this
        // one has to be OBJ.
        (
            "broken-out-of-range.obj",
            "v 0 0 0\nv 1e200 0 0\nv 0 1 0\nf 1 2 3\n",
        ),
        // A missing `endsolid`, which is common and harmless: tolerated, counted.
        (
            "quirk-no-endsolid.stla",
            "solid part\n\
             facet normal 0 0 0\n outer loop\n\
             vertex 0 0 0\n vertex 10 0 0\n vertex 0 10 0\n\
             endloop\n endfacet\n",
        ),
        // Negative indices, relative to the vertex count so far.
        (
            "quirk-negative-indices.obj",
            "v 0 0 0\nv 10 0 0\nv 0 10 0\nf -3 -2 -1\n",
        ),
        // A non-convex face that fan triangulation gets wrong. The U shape is
        // used rather than an L, because an L is star-shaped from every vertex
        // and fans correctly.
        (
            "quirk-nonconvex.obj",
            "v 0 0 0\nv 3 0 0\nv 3 3 0\nv 2 3 0\n\
             v 2 1 0\nv 1 1 0\nv 1 3 0\nv 0 3 0\n\
             f 1 2 3 4 5 6 7 8\n",
        ),
        // An indexed mesh with duplicated coincident vertices. Unwelded it is a
        // pile of disconnected triangles; welding is what restores the topology.
        (
            "quirk-unwelded.obj",
            "v 0 0 0\nv 10 0 0\nv 0 10 0\n\
             v 10 0 0\nv 10 10 0\nv 0 10 0\n\
             f 1 2 3\nf 4 5 6\n",
        ),
    ]
}

#[test]
#[ignore = "writes committed corpus files; run deliberately"]
fn regenerate_corpus() {
    let dir = corpus_dir();
    std::fs::create_dir_all(&dir).expect("corpus directory");
    let mut count = 0;
    for (name, mesh) in good_meshes().into_iter().chain(broken_meshes()) {
        std::fs::write(
            dir.join(format!("{name}.stl")),
            io::stl::write_binary(&mesh),
        )
        .expect("write");
        count += 1;
    }
    for (name, bytes) in malformed_files() {
        std::fs::write(dir.join(format!("{name}.stl")), bytes).expect("write");
        count += 1;
    }
    for (name, text) in text_files() {
        std::fs::write(dir.join(name), text).expect("write");
        count += 1;
    }
    eprintln!("wrote {count} corpus entries to {}", dir.display());
    assert!(count >= 35, "the specification asks for about 35 meshes");
}

/// Loads a corpus file the way the CLI does, returning the welded mesh.
fn load(path: &Path, unit: Unit) -> Result<TriMesh, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("cannot read: {e}"))?;
    let name = path.file_name().and_then(|s| s.to_str());
    let raw = match io::detect(&bytes, name) {
        Format::StlBinary => io::stl::read_binary(&bytes, unit).map_err(|e| e.to_string()),
        Format::StlAscii => {
            io::stl::read_ascii(&String::from_utf8_lossy(&bytes), unit).map_err(|e| e.to_string())
        }
        Format::Obj => {
            io::obj::read(&String::from_utf8_lossy(&bytes), unit).map_err(|e| e.to_string())
        }
        // 3MF states its own unit, so the caller's is only checked for
        // agreement rather than applied.
        Format::ThreeMf => io::threemf::read(&bytes, Some(unit)).map_err(|e| e.to_string()),
    }?;
    weld(&raw, EPS_WELD)
        .map(|(m, _)| m)
        .map_err(|e| e.to_string())
}

#[test]
fn every_corpus_file_has_an_expectation_and_vice_versa() {
    // Guards against the two halves drifting apart, which would otherwise show
    // up as a defect nobody is checking.
    let dir = corpus_dir();
    let expectations: Value = serde_json::from_str(
        &std::fs::read_to_string(dir.join("expectations.json")).expect("read"),
    )
    .expect("valid JSON");
    let listed: BTreeSet<String> = expectations["entries"]
        .as_array()
        .expect("entries array")
        .iter()
        .map(|e| e["file"].as_str().expect("file name").to_owned())
        .collect();

    let on_disk: BTreeSet<String> = std::fs::read_dir(&dir)
        .expect("corpus directory")
        .filter_map(|e| e.ok())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .filter(|n| n != "expectations.json")
        .collect();

    let missing: Vec<&String> = on_disk.difference(&listed).collect();
    let extra: Vec<&String> = listed.difference(&on_disk).collect();
    assert!(
        missing.is_empty(),
        "corpus files with no expectation: {missing:?}"
    );
    assert!(
        extra.is_empty(),
        "expectations with no corpus file: {extra:?}"
    );
    assert!(on_disk.len() >= 35, "only {} corpus entries", on_disk.len());
}

#[test]
fn every_corpus_entry_is_classified_correctly() {
    // The exit criterion: not that the validator runs, but that it reaches the
    // right conclusion about every deliberate defect.
    let dir = corpus_dir();
    let expectations: Value = serde_json::from_str(
        &std::fs::read_to_string(dir.join("expectations.json")).expect("read"),
    )
    .expect("valid JSON");

    let mut checked = 0;
    for entry in expectations["entries"].as_array().expect("entries array") {
        let file = entry["file"].as_str().expect("file name");
        let unit = Unit::from_name(entry["units"].as_str().unwrap_or("mm")).expect("unit");
        let path = dir.join(file);
        let outcome = load(&path, unit);

        match entry["load"].as_str().expect("load field") {
            "reject" => {
                let err = outcome.expect_err(&format!("{file} must be rejected"));
                let needle = entry["error_contains"].as_str().expect("error_contains");
                assert!(
                    err.contains(needle),
                    "{file}: expected an error containing `{needle}`, got `{err}`"
                );
            }
            "ok" => {
                let mesh = outcome.unwrap_or_else(|e| panic!("{file} must load: {e}"));
                let mut report = validate(&mesh);
                if entry.get("self_intersections").is_some() {
                    check_self_intersections(&mesh, &mut report);
                }

                for (field, actual) in [
                    ("is_manifold", report.is_manifold),
                    ("is_watertight", report.is_watertight),
                    (
                        "is_orientation_consistent",
                        report.is_orientation_consistent,
                    ),
                    ("is_solid", report.is_solid()),
                ] {
                    if let Some(expected) = entry.get(field).and_then(Value::as_bool) {
                        assert_eq!(actual, expected, "{file}: {field}");
                    }
                }
                if let Some(expected) = entry.get("components").and_then(Value::as_u64) {
                    assert_eq!(
                        report.components.len() as u64,
                        expected,
                        "{file}: component count"
                    );
                }
                if let Some(expected) = entry.get("triangles").and_then(Value::as_u64) {
                    assert_eq!(u64::from(report.triangles), expected, "{file}: triangles");
                }
                if let Some(expected) = entry.get("vertices").and_then(Value::as_u64) {
                    assert_eq!(u64::from(report.vertices), expected, "{file}: vertices");
                }
                if let Some(expected) = entry.get("genus").and_then(Value::as_i64) {
                    assert_eq!(report.components[0].genus, Some(expected), "{file}: genus");
                }
                if let Some(expected) = entry.get("self_intersections").and_then(Value::as_u64) {
                    let found = report
                        .count_of(chipbreaker_core::mesh::validate::FindingKind::SelfIntersection)
                        as u64;
                    if expected == 0 {
                        assert_eq!(found, 0, "{file}: expected no self-intersections");
                    } else {
                        assert!(
                            found >= expected,
                            "{file}: expected at least {expected} self-intersections, found {found}"
                        );
                    }
                }

                // Finding counts, by kind. Self-intersections are excluded here
                // and handled by the dedicated field above, because their exact
                // count depends on how the two surfaces happen to be
                // tessellated, which is not a property worth pinning.
                let is_self_intersection = |k: chipbreaker_core::mesh::validate::FindingKind| {
                    k == chipbreaker_core::mesh::validate::FindingKind::SelfIntersection
                };
                if let Some(findings) = entry.get("findings").and_then(Value::as_object) {
                    for (kind_name, expected) in findings {
                        let kind = kind_from_name(kind_name);
                        let found = report.count_of(kind) as u64;
                        let expected = expected.as_u64().expect("count");
                        assert_eq!(
                            found,
                            expected,
                            "{file}: expected {expected} `{kind_name}` finding(s), found {found}. \
                             All findings: {:?}",
                            report
                                .findings
                                .iter()
                                .map(|f| f.kind.name())
                                .collect::<Vec<_>>()
                        );
                    }
                    // Nothing unexpected either: any kind not listed must be absent.
                    for f in &report.findings {
                        assert!(
                            findings.contains_key(f.kind.name()) || is_self_intersection(f.kind),
                            "{file}: unexpected `{}` finding: {}",
                            f.kind.name(),
                            f.detail
                        );
                    }
                }
            }
            other => panic!("{file}: unknown load expectation `{other}`"),
        }
        checked += 1;
    }
    assert!(checked >= 35, "only {checked} entries checked");
    eprintln!("{checked} corpus entries classified correctly");
}

fn kind_from_name(name: &str) -> chipbreaker_core::mesh::validate::FindingKind {
    use chipbreaker_core::mesh::validate::FindingKind as K;
    match name {
        "non-manifold-edge" => K::NonManifoldEdge,
        "boundary-edge" => K::BoundaryEdge,
        "inconsistent-orientation" => K::InconsistentOrientation,
        "degenerate-triangle" => K::DegenerateTriangle,
        "duplicate-triangle" => K::DuplicateTriangle,
        "self-intersection" => K::SelfIntersection,
        "inverted-orientation" => K::InvertedOrientation,
        "unused-vertex" => K::UnusedVertex,
        other => panic!("unknown finding kind in expectations: `{other}`"),
    }
}
