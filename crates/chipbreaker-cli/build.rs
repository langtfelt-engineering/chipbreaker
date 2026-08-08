// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Captures build-time facts for the self-test report's `environment` section.
//!
//! Everything recorded here is **excluded from the canonical hash**. The target
//! triple and toolchain version are exactly the things that legitimately differ
//! between the native and WASM runs whose hashes must match; putting them in the
//! hashed section would make the parity check fail by construction.

use std::process::Command;

fn main() {
    // Cargo sets TARGET to the triple being compiled *for*, which is what we
    // want: a wasm32-wasip1 build must report wasm32-wasip1, not the host.
    let target = std::env::var("TARGET").unwrap_or_else(|_| "unknown".to_owned());
    println!("cargo:rustc-env=CHIPBREAKER_TARGET={target}");

    let rustc = std::env::var("RUSTC").unwrap_or_else(|_| "rustc".to_owned());
    let version = Command::new(rustc)
        .arg("--version")
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map_or_else(|| "unknown".to_owned(), |s| s.trim().to_owned());
    println!("cargo:rustc-env=CHIPBREAKER_RUSTC={version}");

    // Without this, a toolchain change would not rebuild and the report would
    // keep claiming the old version.
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=RUSTC");
    println!("cargo:rerun-if-env-changed=TARGET");
}
