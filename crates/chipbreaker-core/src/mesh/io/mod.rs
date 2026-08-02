// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Mesh file formats.
//!
//! Every loader converts to canonical millimetres exactly once, at load, and
//! records the source unit in [`crate::mesh::MeshMeta`] so `mesh inspect` can
//! answer what it assumed. See [`crate::mesh::units`] for why STL and OBJ have
//! no default unit.
