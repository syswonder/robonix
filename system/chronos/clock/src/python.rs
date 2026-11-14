// SPDX-License-Identifier: MulanPSL-2.0

use pyo3::prelude::*;

use crate::{VirtualClockSnapshot, VirtualReferenceTime, VirtualSteadyTime};

#[pyfunction]
/// Read paired reference and local steady time, including reference quality.
fn snapshot(py: Python<'_>) -> PyResult<VirtualClockSnapshot> {
    Ok(py.detach(crate::snapshot)?)
}

#[pyfunction]
/// Read reference time with quality metadata, even when quality is unavailable.
fn reference_time(py: Python<'_>) -> PyResult<VirtualReferenceTime> {
    Ok(py.detach(crate::reference_time)?)
}

#[pyfunction]
/// Read available, nonnegative reference nanoseconds; failures raise OSError.
fn reference_timestamp_ns(py: Python<'_>) -> PyResult<u64> {
    Ok(py.detach(crate::reference_timestamp_ns)?)
}

#[pyfunction]
/// Read local steady nanoseconds independently of reference-clock health.
fn steady_time(py: Python<'_>) -> PyResult<VirtualSteadyTime> {
    Ok(py.detach(crate::steady_time)?)
}

#[pyfunction]
fn wall_time(py: Python<'_>) -> PyResult<VirtualReferenceTime> {
    Ok(py.detach(crate::wall_time)?)
}

#[pyfunction]
fn wall_timestamp_ns(py: Python<'_>) -> PyResult<u64> {
    Ok(py.detach(crate::wall_timestamp_ns)?)
}

#[pyfunction]
#[cfg(target_os = "linux")]
fn configure_reference(py: Python<'_>, kind: &str) -> PyResult<()> {
    let kind = match kind {
        "linux_system" => crate::ReferenceSourceKind::LinuxSystem,
        "ros_time" => crate::ReferenceSourceKind::RosTime,
        _ => {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "reference kind must be linux_system or ros_time",
            ));
        }
    };
    Ok(py.detach(move || {
        crate::configure_reference(crate::ReferenceConfig {
            kind,
            ..Default::default()
        })
    })?)
}

#[pyfunction]
#[cfg(target_os = "linux")]
#[pyo3(signature = (timestamp_ns, generation, available, uncertainty_ns=1, verified=false, reason=String::new()))]
fn update_reference_time(
    py: Python<'_>,
    timestamp_ns: i64,
    generation: u64,
    available: bool,
    uncertainty_ns: u64,
    verified: bool,
    reason: String,
) -> PyResult<()> {
    Ok(py.detach(move || {
        crate::update_reference_time(VirtualReferenceTime {
            timestamp_ns,
            generation,
            generation_changed: false,
            uncertainty_ns,
            available,
            verified,
            reason,
        })
    })?)
}

#[pymodule]
fn robonix_chronos_clock(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<VirtualClockSnapshot>()?;
    module.add_class::<VirtualReferenceTime>()?;
    module.add_class::<VirtualSteadyTime>()?;
    module.add_function(wrap_pyfunction!(snapshot, module)?)?;
    module.add_function(wrap_pyfunction!(reference_time, module)?)?;
    module.add_function(wrap_pyfunction!(reference_timestamp_ns, module)?)?;
    module.add_function(wrap_pyfunction!(steady_time, module)?)?;
    module.add_function(wrap_pyfunction!(wall_time, module)?)?;
    module.add_function(wrap_pyfunction!(wall_timestamp_ns, module)?)?;
    #[cfg(target_os = "linux")]
    {
        module.add_function(wrap_pyfunction!(configure_reference, module)?)?;
        module.add_function(wrap_pyfunction!(update_reference_time, module)?)?;
    }
    Ok(())
}
