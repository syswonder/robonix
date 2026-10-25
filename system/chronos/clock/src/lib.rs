// SPDX-License-Identifier: MulanPSL-2.0

//! Chronos reference and steady clock readings for in-process consumers.
//!
//! Clock reads require no logging, discovery, code generation, or running
//! service. Linux reference readings preserve quality and discontinuities;
//! steady readings remain independent of reference-clock health.

mod config;
#[cfg(feature = "python")]
mod python;
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

static WALL_QUALITY_WARNED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

#[cfg(target_os = "linux")]
struct ReferenceCache {
    latest: Option<(VirtualReferenceTime, u64)>,
    last_available: Option<(u64, i64)>,
    max_uncertainty_ns: u64,
}

#[cfg(target_os = "linux")]
static ROS_REFERENCE: std::sync::LazyLock<std::sync::Mutex<ReferenceCache>> =
    std::sync::LazyLock::new(|| {
        std::sync::Mutex::new(ReferenceCache {
            latest: None,
            last_available: None,
            max_uncertainty_ns: ReferenceConfig::default().max_uncertainty_ns,
        })
    });

/// Select the process-local reference source before starting its consumers.
#[cfg(target_os = "linux")]
pub fn configure_reference(config: ReferenceConfig) -> std::io::Result<()> {
    let max_uncertainty_ns = config.max_uncertainty_ns;
    let clock = VirtualClock::new(config).map_err(std::io::Error::other)?;
    let mut selected = CLOCK
        .lock()
        .map_err(|_| std::io::Error::other("Chronos clock lock poisoned"))?;
    let mut cache = ROS_REFERENCE
        .lock()
        .map_err(|_| std::io::Error::other("Chronos reference lock poisoned"))?;
    cache.latest = None;
    cache.last_available = None;
    cache.max_uncertainty_ns = max_uncertainty_ns;
    *selected = Some(clock);
    Ok(())
}

/// Install a received reference reading without extrapolating from real elapsed time.
#[cfg(target_os = "linux")]
pub fn update_reference_time(mut reference: VirtualReferenceTime) -> std::io::Result<()> {
    if reference.generation == 0 || reference.timestamp_ns < 0 {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "reference requires a nonnegative timestamp and a positive generation",
        ));
    }
    let mut cache = ROS_REFERENCE
        .lock()
        .map_err(|_| std::io::Error::other("Chronos reference lock poisoned"))?;
    if cache
        .latest
        .as_ref()
        .is_some_and(|(latest, _)| reference.generation < latest.generation)
    {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "reference generation regressed",
        ));
    }
    if reference.available && reference.uncertainty_ns > cache.max_uncertainty_ns {
        reference.available = false;
        reference.reason = "reference clock uncertainty exceeds configured limit".into();
    }
    if reference.available
        && cache
            .last_available
            .is_some_and(|(generation, timestamp_ns)| {
                generation == reference.generation && reference.timestamp_ns < timestamp_ns
            })
    {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "reference time regressed without a generation change",
        ));
    }
    let received_ns = reference::read_linux_monotonic_ns().map_err(std::io::Error::other)?;
    // Unavailable readings must not erase the last usable timestamp in a generation.
    if reference.available {
        cache.last_available = Some((reference.generation, reference.timestamp_ns));
    }
    cache.latest = Some((reference, received_ns));
    Ok(())
}

#[cfg(target_os = "linux")]
fn ros_reference_time() -> Result<Option<(VirtualReferenceTime, u64)>, ReferenceError> {
    Ok(ROS_REFERENCE
        .lock()
        .map_err(|_| ReferenceError::Poisoned)?
        .latest
        .clone())
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

/// Return valid wall timestamps despite degraded sampling quality, warning on stderr.
pub fn wall_timestamp_ns() -> std::io::Result<u64> {
    use std::sync::atomic::Ordering;

    let reference = wall_time()?;
    let timestamp = u64::try_from(reference.timestamp_ns).map_err(std::io::Error::other)?;
    if reference.available {
        WALL_QUALITY_WARNED.store(false, Ordering::Relaxed);
    } else if !WALL_QUALITY_WARNED.swap(true, Ordering::Relaxed) {
        use std::io::Write;
        // Logging can read this clock recursively; diagnostics must not fail the read.
        let _ = writeln!(
            std::io::stderr().lock(),
            "Chronos warning: wall clock quality degraded (uncertainty_ns={}): {}; returning timestamp",
            reference.uncertainty_ns,
            reference.reason,
        );
    }
    Ok(timestamp)
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

#[cfg(all(test, target_os = "linux"))]
mod tests {
    #[test]
    fn poisoned_reference_lock_is_not_a_clock_read_error() {
        let result = std::thread::spawn(|| {
            let _cache = super::ROS_REFERENCE.lock().unwrap();
            panic!("poison the reference cache");
        })
        .join();
        assert!(result.is_err());
        assert_eq!(
            super::ros_reference_time().unwrap_err(),
            super::ReferenceError::Poisoned
        );
    }
}
