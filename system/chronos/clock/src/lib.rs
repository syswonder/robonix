// SPDX-License-Identifier: MulanPSL-2.0

//! Chronos reference and steady clock readings for in-process consumers.
//!
//! Clock reads require no logging, discovery, code generation, or running
//! service. Linux reference readings preserve quality and discontinuities;
//! steady readings remain independent of reference-clock health.

mod config;
mod reference;
mod virtual_clock;

pub use config::{ChronosConfigError, ReferenceConfig, ReferenceSourceKind};
#[cfg(target_os = "linux")]
pub use reference::LinuxSystemReference;
pub use reference::{ReferenceError, ReferenceSample, ReferenceStatus};
#[cfg(target_os = "linux")]
pub use virtual_clock::VirtualClock;
pub use virtual_clock::{VirtualClockSnapshot, VirtualReferenceTime, VirtualSteadyTime};

#[cfg(target_os = "linux")]
static CLOCK: std::sync::Mutex<Option<VirtualClock>> = std::sync::Mutex::new(None);

#[cfg(target_os = "linux")]
static WALL: std::sync::Mutex<Option<VirtualClock>> = std::sync::Mutex::new(None);

#[cfg(target_os = "linux")]
static ROS_REFERENCE: std::sync::Mutex<Option<VirtualReferenceTime>> = std::sync::Mutex::new(None);

/// Select the process-local reference source before starting its consumers.
#[cfg(target_os = "linux")]
pub fn configure_reference(config: ReferenceConfig) -> std::io::Result<()> {
    let clock = VirtualClock::new(config).map_err(std::io::Error::other)?;
    let mut selected = CLOCK
        .lock()
        .map_err(|_| std::io::Error::other("Chronos clock lock poisoned"))?;
    *ROS_REFERENCE
        .lock()
        .map_err(|_| std::io::Error::other("Chronos reference lock poisoned"))? = None;
    *selected = Some(clock);
    Ok(())
}

/// Install a received reference reading without extrapolating from real elapsed time.
#[cfg(target_os = "linux")]
pub fn update_reference_time(reference: VirtualReferenceTime) -> std::io::Result<()> {
    if reference.generation == 0 || reference.timestamp_ns < 0 {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "reference requires a nonnegative timestamp and a positive generation",
        ));
    }
    let mut latest = ROS_REFERENCE
        .lock()
        .map_err(|_| std::io::Error::other("Chronos reference lock poisoned"))?;
    if reference.available
        && latest.as_ref().is_some_and(|previous| {
            previous.generation == reference.generation
                && reference.timestamp_ns < previous.timestamp_ns
        })
    {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "reference time regressed without a generation change",
        ));
    }
    *latest = Some(reference);
    Ok(())
}

#[cfg(target_os = "linux")]
fn ros_reference_time() -> Result<VirtualReferenceTime, ReferenceError> {
    Ok(ROS_REFERENCE
        .lock()
        .map_err(|_| ReferenceError::ClockRead {
            errno: libc::EOWNERDEAD,
        })?
        .clone()
        .unwrap_or(VirtualReferenceTime {
            timestamp_ns: 0,
            generation: 1,
            generation_changed: false,
            uncertainty_ns: 0,
            available: false,
            verified: false,
            reason: "/clock has not supplied reference time".into(),
        }))
}

/// Read Linux wall time independently of the selected robot reference source.
pub fn wall_time() -> std::io::Result<VirtualReferenceTime> {
    #[cfg(target_os = "linux")]
    {
        Ok(snapshot_from(&WALL, "Chronos wall clock lock poisoned")?.reference)
    }
    #[cfg(not(target_os = "linux"))]
    Err(std::io::Error::new(
        std::io::ErrorKind::Unsupported,
        "Chronos clock reads require Linux",
    ))
}

pub fn wall_timestamp_ns() -> std::io::Result<u64> {
    available_timestamp(wall_time()?)
}

/// Read a paired snapshot from the selected process-local reference.
///
/// Initialization and clock reads never log. Poisoned ownership, unsupported
/// platforms, and failed reads return errors; no timestamp is synthesized.
/// Linux generations belong to the process-local observer; ROS generations come from the input.
pub fn snapshot() -> std::io::Result<VirtualClockSnapshot> {
    #[cfg(target_os = "linux")]
    {
        snapshot_from(&CLOCK, "Chronos clock lock poisoned")
    }
    #[cfg(not(target_os = "linux"))]
    {
        Err(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "Chronos clock reads require Linux",
        ))
    }
}

#[cfg(target_os = "linux")]
fn snapshot_from(
    clock: &std::sync::Mutex<Option<VirtualClock>>,
    poison_reason: &str,
) -> std::io::Result<VirtualClockSnapshot> {
    let mut clock = clock
        .lock()
        .map_err(|_| std::io::Error::other(poison_reason.to_owned()))?;
    if clock.is_none() {
        *clock =
            Some(VirtualClock::new(ReferenceConfig::default()).map_err(std::io::Error::other)?);
    }
    clock
        .as_mut()
        .expect("initialized clock")
        .snapshot()
        .map_err(std::io::Error::other)
}

/// Read selected reference time and quality from the process-local clock.
pub fn reference_time() -> std::io::Result<VirtualReferenceTime> {
    Ok(snapshot()?.reference)
}

/// Read an available, nonnegative timestamp in the selected reference domain.
/// Unverified Linux time is accepted; read, quality and range failures remain errors.
pub fn reference_timestamp_ns() -> std::io::Result<u64> {
    available_timestamp(reference_time()?)
}

fn available_timestamp(reference: VirtualReferenceTime) -> std::io::Result<u64> {
    if !reference.available {
        return Err(std::io::Error::other(format!(
            "Chronos reference unavailable: {}",
            reference.reason
        )));
    }
    u64::try_from(reference.timestamp_ns).map_err(std::io::Error::other)
}

/// Read local steady time independently of reference health or ownership.
pub fn steady_time() -> std::io::Result<VirtualSteadyTime> {
    #[cfg(target_os = "linux")]
    {
        Ok(VirtualSteadyTime {
            timestamp_ns: reference::read_linux_monotonic_ns().map_err(std::io::Error::other)?,
        })
    }
    #[cfg(not(target_os = "linux"))]
    {
        Err(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "Chronos clock reads require Linux",
        ))
    }
}
