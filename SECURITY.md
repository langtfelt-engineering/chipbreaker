# Security policy

## Reporting a vulnerability

**Please do not open a public issue for a security problem.**

Use GitHub's [private vulnerability reporting][report] on this repository.

[report]: https://github.com/langtfelt-engineering/chipbreaker/security/advisories/new

It is the only channel for security reports: it is private by construction, it
keeps the whole exchange attached to the repository, and it turns into a
published advisory when the fix ships.

What helps, in rough order of usefulness:

1. **An input file that reproduces it.** A malformed `.stl`, `.nc`, `.tdx` or
   tool library is worth more than a description of one.
2. The exact command, and the commit or version.
3. What you expected instead.

We will acknowledge within **five working days** and tell you what we think the
severity is and what we intend to do. If we disagree with your assessment we will
say so and why, rather than going quiet. If you want credit in the advisory, say
so; if you would rather not be named, that is fine too.

There is no bug bounty. This is a small project and we would rather be honest
about that up front than imply otherwise.

## What is supported

Pre-1.0. **Only `main` is supported.** There are no released versions yet and no
backport branches, so a fix lands on `main` and that is the whole of it. When
tagged releases begin, this section will say which of them are maintained.

## What the threat model actually is

Chipbreaker is a library and a command-line tool. It has no network code, opens
no sockets, runs no server, and executes nothing it reads. The realistic exposure
is **parsing input you did not write**:

| Input | Format |
|---|---|
| NC programs | RS-274 text |
| Meshes | binary and ASCII STL, OBJ, 3MF (a ZIP container with XML inside) |
| Fields | `.dexel` and `.tdx`, raw IEEE-754 bit patterns |
| Tool libraries | JSON |

A machinist opening a supplier's STL, or a shop running Chipbreaker over
customer-supplied G-code, is the case worth protecting.

### What we consider a vulnerability

- **Memory unsafety of any kind.** Every crate carries
  `#![forbid(unsafe_code)]`, enforced in CI, so this should be impossible in our
  own code — which makes any instance of it a serious finding, most likely in a
  dependency.
- **Unbounded resource consumption from a small input.** A 2 KB mesh that makes
  the process allocate gigabytes, or spin without terminating, is a denial of
  service against anyone running this in a pipeline. The `mem-estimate` command
  and the memory ceiling exist partly to make allocation predictable and
  refusable; a way around them counts.
- **Path traversal or unintended writes** from a crafted archive. 3MF is a ZIP,
  and ZIP entries can carry hostile paths. The reader is read-only and extracts
  nothing to disk — it looks up the model part by name inside the archive — so we
  believe the surface is closed, which is exactly the sort of belief worth
  testing.
- **A silently wrong answer caused by a crafted input.** This one is unusual to
  find in a security policy, and it belongs here: Chipbreaker is a verification
  tool, so an input that makes it report a part as clean when it is gouged is a
  worse outcome than a crash. If you find one, treat it as a security issue and
  we will.

### What we do not consider a vulnerability

- **A panic on a malformed file.** We would like to know about it and we will fix
  it — please file it as an ordinary issue — but a Rust panic unwinds or aborts;
  it does not corrupt memory or run anything.

  We hunt these deliberately rather than waiting for them:
  [`tests/corpus/mesh/`](tests/corpus/mesh/) holds sixteen meshes broken in
  sixteen different ways, each with its expected diagnosis, and the G-code parser
  and the interval algebra are fuzzed against hostile input nightly. The mesh
  readers are **not** fuzzed yet — they have the corpus and no generator — so
  that is the surface where a report is most likely to tell us something we do
  not already know.
- **Slowness on a genuinely large job.** A million-segment program over a fine
  lattice is meant to take a long time. Resource use grossly disproportionate to
  the input is a different matter; see above.
- **Anything that requires the attacker to already control the machine** running
  Chipbreaker.

## What this policy does not cover

Chipbreaker verifies a **program** against an **ideal geometric cutting model**.
It does not model tool wear, deflection, thermal growth, spindle runout, backlash
or how a controller interpolates between the points it is given.

A part can match the simulation exactly and still be out of tolerance for any of
those reasons. That is a limit of scope, stated everywhere it matters, and it is
not a security issue — but it is the misunderstanding most likely to cause real
harm with this tool, so it is written here too.

**Chipbreaker is not a safety interlock.** Nothing in it should be the only thing
standing between a program and a machine.
