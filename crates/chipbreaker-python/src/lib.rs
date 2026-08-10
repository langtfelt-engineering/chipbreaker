// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

#![forbid(unsafe_code)]

//! Python bindings.
//!
//! # Why these bind the Rust core rather than the C ABI
//!
//! Both were available. Binding the C ABI would have meant marshalling every
//! input into raw pointers, receiving a JSON string back, and parsing it in
//! Rust to hand Python a dictionary — three conversions to reach a place both
//! layers can already stand.
//!
//! More importantly, it would have made the C ABI's *shape* into Python's
//! shape. The C ABI has opaque handles and explicit frees because C has no
//! other way to manage a lifetime; Python has garbage collection and
//! exceptions, and inheriting a C ownership model into it would produce a
//! library that reads like a translation.
//!
//! What matters is that the two agree about **answers**, and they do, because
//! both call the same [`chipbreaker_gcode::pipeline`] assembly. The C ABI is
//! itself a thin shell over that function. These bindings are a second thin
//! shell over the same one, so the two are peers rather than a stack, and
//! neither can drift from the other without the shared assembly changing.
//!
//! # The sentence has to survive
//!
//! A refusal raises [`Refused`], and the exception's string is the engine's own
//! sentence, unmodified. A Python user who writes `except Refused as e:
//! print(e)` sees exactly what a CLI user sees. That is the whole requirement:
//! the reason a job was declined is the most useful thing this engine produces,
//! and a binding that turned it into `RuntimeError("simulation failed")` would
//! have discarded it while appearing to work.

use pyo3::exceptions::{PyOSError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict, PyList};

pyo3::create_exception!(
    _chipbreaker,
    Refused,
    pyo3::exceptions::PyException,
    "The engine declined the job, and this is why.\n\n\
     Raised for every designed refusal: an unsupported construct, a program in \
     another language, a macro the engine will not guess at, a resolution that \
     will not fit in the memory allowed. `str(exc)` is the engine's own \
     sentence and usually names what to do instead.\n\n\
     **This is not an error in the usual sense.** The engine answered the \
     question; the answer is no. Catching it and showing the message is the \
     correct handling, and logging it as a failure to retry is not -- nothing \
     about the input will have changed."
);

/// Runs one verification job.
///
/// Returns the report as a `dict`, parsed rather than as a JSON string: a
/// caller that wanted text can call `json.dumps`, and one that wanted values
/// should not have to parse to get them.
///
/// Raises [`Refused`] when the engine declines, `ValueError` when an argument
/// is not usable, and `OSError` when a file cannot be read.
#[pyfunction]
#[pyo3(signature = (
    program,
    tools,
    stock_stl,
    *,
    nominal_stl = None,
    tool = None,
    source = None,
    resolution_mm = 0.5,
    tolerance_mm = 0.1,
    clearance_mm = 0.0,
    memory_ceiling_bytes = None,
))]
#[allow(clippy::too_many_arguments, reason = "the job's inputs, all keyword")]
fn run<'py>(
    py: Python<'py>,
    program: &str,
    tools: &str,
    stock_stl: &[u8],
    nominal_stl: Option<&[u8]>,
    tool: Option<&str>,
    source: Option<&str>,
    resolution_mm: f64,
    tolerance_mm: f64,
    clearance_mm: f64,
    memory_ceiling_bytes: Option<u64>,
) -> PyResult<Bound<'py, PyDict>> {
    let request = chipbreaker_gcode::pipeline::JobRequest {
        program,
        tools,
        stock_stl,
        nominal_stl,
        tool_id: tool,
        source,
        resolution_mm,
        tolerance_mm,
        clearance_mm,
        memory_ceiling_bytes,
        segment_cap: None,
    };

    // The GIL is released for the duration of the run. A verification takes
    // seconds on a real part, and holding the interpreter for that would make
    // this unusable from any program that also has a user interface.
    let outcome = py.detach(|| chipbreaker_gcode::pipeline::run(&request));

    match outcome {
        Ok(report) => to_dict(py, &report.to_json()),
        // Unmodified. Not prefixed, not wrapped, not truncated.
        Err(why) => Err(Refused::new_err(why)),
    }
}

/// The engine's self-test digest.
///
/// Identical on every target the engine builds for, including WebAssembly,
/// which makes it a statement about behaviour rather than about a build. The
/// first call runs the suites and takes on the order of a second and a half.
#[pyfunction]
fn selftest_digest() -> String {
    selftest().digest.to_hex()
}

/// Whether every self-test suite passes in this build.
#[pyfunction]
fn selftest_passed() -> bool {
    selftest().passed()
}

/// How many self-test cases ran.
#[pyfunction]
fn selftest_case_count() -> usize {
    selftest().suites.iter().map(|s| s.cases).sum()
}

/// The engine version.
#[pyfunction]
fn engine_version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}

fn selftest() -> &'static chipbreaker_core::selftest::SelfTestReport {
    use std::sync::OnceLock;
    static REPORT: OnceLock<chipbreaker_core::selftest::SelfTestReport> = OnceLock::new();
    REPORT
        .get_or_init(|| chipbreaker_core::selftest::run_with(chipbreaker_gcode::selftest::suites()))
}

/// Converts a report to Python values.
///
/// Numbers stay numbers and lists stay lists. Handing back a JSON string would
/// have been three lines shorter and would have made every caller parse a
/// document this function already had in front of it.
fn to_dict<'py>(py: Python<'py>, value: &serde_json::Value) -> PyResult<Bound<'py, PyDict>> {
    match to_py(py, value)?.cast_into::<PyDict>() {
        Ok(d) => Ok(d),
        Err(_) => Err(PyValueError::new_err(
            "the engine produced a report that was not an object; this is a defect",
        )),
    }
}

fn to_py<'py>(py: Python<'py>, value: &serde_json::Value) -> PyResult<Bound<'py, PyAny>> {
    Ok(match value {
        serde_json::Value::Null => py.None().into_bound(py),
        serde_json::Value::Bool(b) => b.into_pyobject(py)?.to_owned().into_any(),
        serde_json::Value::Number(n) => {
            if let Some(i) = n.as_u64() {
                i.into_pyobject(py)?.into_any()
            } else if let Some(i) = n.as_i64() {
                i.into_pyobject(py)?.into_any()
            } else {
                // Every remaining number in this schema is an f64, and this
                // engine is f64 throughout. `as_f64` cannot fail here, and NaN
                // cannot occur because serde_json will not serialise one.
                n.as_f64().unwrap_or(0.0).into_pyobject(py)?.into_any()
            }
        }
        serde_json::Value::String(s) => s.into_pyobject(py)?.into_any(),
        serde_json::Value::Array(a) => {
            let list = PyList::empty(py);
            for v in a {
                list.append(to_py(py, v)?)?;
            }
            list.into_any()
        }
        serde_json::Value::Object(o) => {
            let dict = PyDict::new(py);
            for (k, v) in o {
                dict.set_item(k, to_py(py, v)?)?;
            }
            dict.into_any()
        }
    })
}

/// Reads a file, raising `OSError` the way Python does.
///
/// Present so the quick-start does not have to explain the difference between
/// bytes and text for STL, which is a question about the format rather than
/// about this library.
#[pyfunction]
fn read_bytes<'py>(py: Python<'py>, path: &str) -> PyResult<Bound<'py, PyBytes>> {
    match std::fs::read(path) {
        Ok(b) => Ok(PyBytes::new(py, &b)),
        Err(e) => Err(PyOSError::new_err(format!("{path}: {e}"))),
    }
}

#[pymodule]
fn _chipbreaker(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("Refused", m.py().get_type::<Refused>())?;
    m.add_function(wrap_pyfunction!(run, m)?)?;
    m.add_function(wrap_pyfunction!(read_bytes, m)?)?;
    m.add_function(wrap_pyfunction!(selftest_digest, m)?)?;
    m.add_function(wrap_pyfunction!(selftest_passed, m)?)?;
    m.add_function(wrap_pyfunction!(selftest_case_count, m)?)?;
    m.add_function(wrap_pyfunction!(engine_version, m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
