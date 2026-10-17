// SPDX-License-Identifier: MulanPSL-2.0

use robonix_chronos::{
    ChronosCore, ChronosError, ClockDomainDescriptor, ClockDomainPolicy,
    ClockModelParametersSnapshot, ClockObservation, DomainState, ModelUpdatePolicy,
};

/// Build the common descriptor while varying declared counter wrap behavior.
fn descriptor(wrap_period_ns: Option<u64>) -> ClockDomainDescriptor {
    ClockDomainDescriptor {
        domain_id: "camera_clock".to_string(),
        resolution_ns: 1,
        wrap_period_ns,
    }
}

/// Build policy limits shared by core behavior tests.
fn policy() -> ClockDomainPolicy {
    ClockDomainPolicy {
        minimum_observations: 3,
        observation_window: 8,
        max_drift_ppm: 500,
        max_residual_ns: 2_000,
        tracking_timeout_ns: 1_000_000_000,
        holdover_duration_ns: 3_000_000_000,
        max_uncertainty_ns: 2_000_000,
        model_update: ModelUpdatePolicy::Step {},
    }
}

/// Build one paired observation with a deterministic sequence and receipt time.
fn observation(sequence: u64, device: i64, reference: i64, steady: u64) -> ClockObservation {
    ClockObservation {
        sequence_id: sequence,
        device_timestamp_ns: device,
        reference_timestamp_ns: reference,
        uncertainty_ns: 100,
        steady_received_ns: steady,
    }
}

/// Apply Chronos's nearest-integer, half-away-from-zero division rule.
fn divide_rounded(numerator: i128, denominator: i128) -> i128 {
    let quotient = numerator / denominator;
    let remainder = numerator % denominator;
    let twice = remainder.abs() * 2;
    if twice < denominator {
        quotient
    } else if numerator.is_negative() {
        quotient - 1
    } else {
        quotient + 1
    }
}

/// Reconstruct one local conversion using only published model parameters.
fn convert_published(model: &ClockModelParametersSnapshot, raw: i64) -> i64 {
    let mut normalized = i128::from(raw) + i128::from(model.wrap_offset_ns);
    if let Some(period) = model.wrap_period_ns {
        let period = i128::from(period);
        let difference = i128::from(raw) - i128::from(model.latest_raw_device_ns);
        if difference < -(period / 2) {
            normalized += period;
        } else if difference > period / 2 {
            normalized -= period;
        }
    }
    let delta = normalized - i128::from(model.device_anchor_ns);
    let rate = divide_rounded(delta * i128::from(model.rate_adjust_ppb), 1_000_000_000);
    i64::try_from(
        i128::from(model.reference_anchor_ns) + delta + rate + i128::from(model.residual_offset_ns),
    )
    .unwrap()
}

/// Feed exact samples for a clock whose signed rate correction is known.
fn tracking_core(rate_adjust_ppb: i64) -> ChronosCore {
    let mut core = ChronosCore::new();
    core.register(descriptor(None), policy()).unwrap();
    for sequence in 1..=4 {
        let device = sequence as i64 * 1_000_000_000;
        let correction = device * rate_adjust_ppb / 1_000_000_000;
        core.observe(
            "camera_clock",
            observation(
                sequence,
                device,
                10_000_000_000 + device + correction,
                sequence,
            ),
        )
        .unwrap();
    }
    core
}

/// Fit a signed-rate model across one declared counter wrap.
fn wrapping_tracking_core(rate_adjust_ppb: i64) -> ChronosCore {
    let mut core = ChronosCore::new();
    core.register(descriptor(Some(1_000_000)), policy())
        .unwrap();
    for (sequence, raw, normalized) in [
        (1, 700_000, 700_000),
        (2, 800_000, 800_000),
        (3, 900_000, 900_000),
        (4, 20_000, 1_020_000),
    ] {
        let correction = divide_rounded(
            i128::from(normalized) * i128::from(rate_adjust_ppb),
            1_000_000_000,
        );
        core.observe(
            "camera_clock",
            observation(
                sequence,
                raw,
                10_000_000 + normalized + i64::try_from(correction).unwrap(),
                sequence,
            ),
        )
        .unwrap();
    }
    core
}

/// Select an in-range raw value whose signed rate term must be rounded.
fn raw_requiring_rounding(
    model: &robonix_chronos::ClockModel,
    published: &ClockModelParametersSnapshot,
    candidates: impl Iterator<Item = i64>,
) -> i64 {
    candidates
        .filter(|raw| model.convert(*raw).is_ok())
        .find(|raw| {
            let normalized = i128::from(*raw) + i128::from(published.wrap_offset_ns);
            let delta = normalized - i128::from(published.device_anchor_ns);
            let remainder = (delta * i128::from(published.rate_adjust_ppb)) % 1_000_000_000;
            remainder.abs() * 2 > 1_000_000_000
        })
        .expect("test model must expose a value with a non-trivial fractional rate term")
}

#[test]
/// Both oscillator directions must retain their signed frequency correction.
fn estimates_positive_and_negative_drift() {
    let fast = tracking_core(125_000);
    assert_eq!(
        fast.model("camera_clock").unwrap().rate_adjust_ppb(),
        125_000
    );

    let slow = tracking_core(-80_000);
    assert_eq!(
        slow.model("camera_clock").unwrap().rate_adjust_ppb(),
        -80_000
    );
}

#[test]
/// Published coefficients reproduce non-integral positive and negative drift.
fn published_parameters_preserve_conversion_rounding() {
    for rate_adjust_ppb in [125_000, -80_000] {
        let core = tracking_core(rate_adjust_ppb);
        let model = core.model("camera_clock").unwrap();
        let published = ClockModelParametersSnapshot::from(&model);
        let raw = raw_requiring_rounding(
            &model,
            &published,
            published.device_anchor_ns..=published.device_anchor_ns + 20_000,
        );
        assert_eq!(
            convert_published(&published, raw),
            model.convert(raw).unwrap().reference_timestamp_ns
        );

        let wrapping = wrapping_tracking_core(rate_adjust_ppb);
        let model = wrapping.model("camera_clock").unwrap();
        let published = ClockModelParametersSnapshot::from(&model);
        let raw = raw_requiring_rounding(&model, &published, 0..=20_000);
        assert_eq!(
            convert_published(&published, raw),
            model.convert(raw).unwrap().reference_timestamp_ns
        );
    }
}

#[test]
/// Conversion carries the generation and version of the model that was used.
fn converts_with_versioned_checked_model() {
    let core = tracking_core(125_000);
    let converted = core.convert("camera_clock", 4_500_000_000).unwrap();
    assert_eq!(converted.reference_timestamp_ns, 14_500_562_500);
    assert_eq!(converted.generation, 1);
    assert_eq!(converted.reference_generation, 1);
    assert!(converted.model_version >= 1);
    assert_eq!(converted.state, DomainState::Tracking);
    assert!(converted.within_validity);
    assert_eq!(converted.last_observation_steady_ns, 4);
}

#[test]
/// A mature model rejects a large residual without replacing its model version.
fn rejects_obvious_outlier_without_replacing_model() {
    let mut core = tracking_core(0);
    let version = core.model("camera_clock").unwrap().model_version();
    let error = core
        .observe(
            "camera_clock",
            observation(5, 5_000_000_000, 99_000_000_000, 5),
        )
        .unwrap_err();
    assert!(matches!(error, ChronosError::ObservationOutlier { .. }));
    assert_eq!(core.model("camera_clock").unwrap().model_version(), version);
    assert_eq!(
        core.status("camera_clock", 5)
            .unwrap()
            .rejected_observations,
        1
    );
}

#[test]
/// An undeclared device-time regression invalidates the previous generation.
fn device_regression_starts_a_new_generation() {
    let mut core = tracking_core(0);
    let status = core
        .observe("camera_clock", observation(5, 10, 20_000_000_010, 5))
        .unwrap();
    assert_eq!(status.generation, 2);
    assert_eq!(status.state, DomainState::Reset);
    assert!(matches!(
        core.model("camera_clock"),
        Err(ChronosError::ModelUnavailable(DomainState::Reset))
    ));
}

#[test]
/// A restarted sequence requires explicit reset so late samples stay rejected.
fn implicit_reset_rejects_restarted_sequence() {
    let mut core = tracking_core(0);
    assert_eq!(
        core.observe("camera_clock", observation(0, 10, 20_000_000_010, 5)),
        Err(ChronosError::ObservationOutOfOrder)
    );
    assert_eq!(core.status("camera_clock", 5).unwrap().generation, 1);
}

#[test]
/// A declared counter wrap extends time without changing generation.
fn declared_counter_wrap_keeps_the_generation() {
    let mut core = ChronosCore::new();
    core.register(descriptor(Some(1_000)), policy()).unwrap();
    core.observe("camera_clock", observation(1, 800, 10_000, 1))
        .unwrap();
    core.observe("camera_clock", observation(2, 900, 10_100, 2))
        .unwrap();
    let status = core
        .observe("camera_clock", observation(3, 20, 10_220, 3))
        .unwrap();
    assert_eq!(status.generation, 1);
    assert_eq!(status.state, DomainState::Tracking);
    assert_eq!(
        core.convert("camera_clock", 30)
            .unwrap()
            .reference_timestamp_ns,
        10_230
    );
    let model = core.model("camera_clock").unwrap();
    let published = ClockModelParametersSnapshot::from(&model);
    assert_eq!(published.wrap_period_ns, Some(1_000));
    assert_eq!(published.latest_raw_device_ns, 20);
    assert_eq!(published.wrap_offset_ns, 1_000);
    assert_eq!(convert_published(&published, 30), 10_230);
}

#[test]
/// Stale models remain calculable while reporting degraded quality.
fn stale_observations_enter_holdover_then_unsynced() {
    let mut core = tracking_core(0);
    core.refresh(2_000_000_004).unwrap();
    let holdover = core.status("camera_clock", 2_000_000_004).unwrap();
    assert_eq!(holdover.state, DomainState::Holdover);
    assert!(holdover.uncertainty_ns.unwrap() > 100);

    core.refresh(4_000_000_005).unwrap();
    let unsynced = core.status("camera_clock", 4_000_000_005).unwrap();
    assert_eq!(unsynced.state, DomainState::Unsynced);
    let estimate = core.convert("camera_clock", 4_000_000_000).unwrap();
    assert_eq!(estimate.state, DomainState::Unsynced);
    assert!(estimate.within_validity);

    let previous_uncertainty = estimate.uncertainty_ns;
    core.refresh(5_000_000_005).unwrap();
    assert!(
        core.convert("camera_clock", 4_000_000_000)
            .unwrap()
            .uncertainty_ns
            > previous_uncertainty
    );

    let extrapolated = core.convert("camera_clock", 9_000_000_000).unwrap();
    assert!(!extrapolated.within_validity);
    assert_eq!(extrapolated.uncertainty_ns, u64::MAX);
}

#[test]
/// Reference loss retains conversion coefficients with explicit bad quality.
fn unavailable_reference_degrades_and_recovers_model() {
    let mut core = tracking_core(0);
    let tracking_version = core.model("camera_clock").unwrap().model_version();
    core.update_reference_availability(false).unwrap();
    core.refresh(2_000_000_004).unwrap();

    let status = core.status("camera_clock", 2_000_000_004).unwrap();
    assert_eq!(status.state, DomainState::Unsynced);
    assert_eq!(status.uncertainty_ns, Some(u64::MAX));
    let estimate = core.convert("camera_clock", 4_500_000_000).unwrap();
    assert_eq!(estimate.state, DomainState::Unsynced);
    assert_eq!(estimate.uncertainty_ns, u64::MAX);
    let unavailable_model = core.model("camera_clock").unwrap();
    assert_eq!(unavailable_model.state(), DomainState::Unsynced);
    assert_eq!(unavailable_model.uncertainty_ns(), u64::MAX);
    assert!(unavailable_model.model_version() > tracking_version);
    assert_eq!(
        core.observe(
            "camera_clock",
            observation(5, 5_000_000_000, 15_000_000_000, 5),
        ),
        Err(ChronosError::ReferenceUnavailable)
    );

    core.update_reference_availability(true).unwrap();
    let recovered_version = core.model("camera_clock").unwrap().model_version();
    assert!(recovered_version > unavailable_model.model_version());
    let recovered = core
        .observe(
            "camera_clock",
            observation(5, 5_000_000_000, 15_000_000_000, 5),
        )
        .unwrap();
    assert_eq!(recovered.state, DomainState::Tracking);
    assert_ne!(recovered.uncertainty_ns, Some(u64::MAX));
}

#[test]
/// Duplicate sequences and decreasing monotonic receipt times are rejected.
fn ordering_uses_sequence_and_monotonic_receipt_time() {
    let mut core = ChronosCore::new();
    core.register(descriptor(None), policy()).unwrap();
    core.observe("camera_clock", observation(1, 10, 20, 10))
        .unwrap();
    assert_eq!(
        core.observe("camera_clock", observation(1, 11, 21, 11)),
        Err(ChronosError::ObservationOutOfOrder)
    );
    assert_eq!(
        core.observe("camera_clock", observation(2, 11, 21, 9)),
        Err(ChronosError::ObservationOutOfOrder)
    );
}

#[test]
/// Removing a domain also removes its status and model history.
fn unregister_removes_domain_state() {
    let mut core = ChronosCore::new();
    core.register(descriptor(None), policy()).unwrap();
    core.unregister("camera_clock").unwrap();
    assert_eq!(
        core.status("camera_clock", 0),
        Err(ChronosError::UnknownDomain("camera_clock".to_string()))
    );
}

#[test]
/// A new reference generation retains the old estimate with explicit identity.
fn reference_generation_resets_registered_domains() {
    let mut core = tracking_core(0);
    assert_eq!(core.reference_generation(), 1);
    core.update_reference_generation(2).unwrap();
    assert_eq!(core.reference_generation(), 2);
    let reset = core.status("camera_clock", 4).unwrap();
    assert_eq!(reset.generation, 2);
    assert_eq!(reset.reference_generation, 2);
    assert_eq!(reset.state, DomainState::Reset);
    assert_eq!(reset.uncertainty_ns, None);
    let estimate = core.convert("camera_clock", 4_500_000_000).unwrap();
    assert_eq!(estimate.generation, 1);
    assert_eq!(estimate.reference_generation, 1);
    assert_eq!(estimate.state, DomainState::Unsynced);
    core.update_reference_generation(2).unwrap();
    assert_eq!(core.status("camera_clock", 4).unwrap().generation, 2);
    assert!(matches!(
        core.update_reference_generation(1),
        Err(ChronosError::ReferenceGenerationOutOfOrder { .. })
    ));
}

#[test]
/// Confirmed reset creates a generation before accepting restarted counters.
fn explicit_reset_allows_device_sequence_to_restart() {
    let mut core = tracking_core(0);
    let reset = core.reset("camera_clock").unwrap();
    assert_eq!(reset.generation, 2);
    assert_eq!(reset.state, DomainState::Reset);

    let status = core
        .observe("camera_clock", observation(0, 10, 20_000_000_010, 10))
        .unwrap();
    assert_eq!(status.generation, 2);
    assert_eq!(status.state, DomainState::Reset);
}

#[test]
/// A rejected first sample after wrap must not consume the wrap transition.
fn rejected_wrap_sample_does_not_advance_wrap_epoch() {
    let mut core = ChronosCore::new();
    let mut wrap_policy = policy();
    wrap_policy.max_residual_ns = 5;
    core.register(descriptor(Some(1_000)), wrap_policy).unwrap();
    core.observe("camera_clock", observation(1, 700, 10_000, 1))
        .unwrap();
    core.observe("camera_clock", observation(2, 800, 10_100, 2))
        .unwrap();
    core.observe("camera_clock", observation(3, 900, 10_200, 3))
        .unwrap();

    assert!(matches!(
        core.observe("camera_clock", observation(4, 20, 99_999, 4)),
        Err(ChronosError::ObservationOutlier { .. })
    ));
    let status = core
        .observe("camera_clock", observation(5, 30, 10_330, 5))
        .unwrap();
    assert_eq!(status.generation, 1);
    assert_eq!(
        core.convert("camera_clock", 40)
            .unwrap()
            .reference_timestamp_ns,
        10_340
    );
}

#[test]
/// Rejected inputs still consume their ordering identity to prevent replay.
fn rejected_observation_advances_input_ordering_boundary() {
    let mut core = tracking_core(0);
    assert!(matches!(
        core.observe(
            "camera_clock",
            observation(5, 5_000_000_000, 99_000_000_000, 5),
        ),
        Err(ChronosError::ObservationOutlier { .. })
    ));
    assert_eq!(
        core.observe(
            "camera_clock",
            observation(5, 5_000_000_001, 15_000_000_001, 6),
        ),
        Err(ChronosError::ObservationOutOfOrder)
    );
    assert_eq!(
        core.status("camera_clock", 6)
            .unwrap()
            .rejected_observations,
        2
    );
    assert_eq!(
        core.status("camera_clock", 4),
        Err(ChronosError::ObservationOutOfOrder)
    );
}

#[test]
/// Repeated device timestamps are rejected as input, not treated as core faults.
fn repeated_device_timestamp_is_rejected_without_fault() {
    let mut core = tracking_core(0);
    let version = core.model("camera_clock").unwrap().model_version();
    assert!(matches!(
        core.observe(
            "camera_clock",
            observation(5, 4_000_000_000, 14_000_000_000, 5),
        ),
        Err(ChronosError::InvalidObservation(_))
    ));
    let status = core.status("camera_clock", 5).unwrap();
    assert_eq!(status.state, DomainState::Tracking);
    assert_eq!(status.model_version, version);
}

#[test]
/// Wrapping conversion rejects noncanonical and exact-half-period timestamps.
fn wrapping_conversion_rejects_ambiguous_inputs() {
    let mut core = ChronosCore::new();
    core.register(descriptor(Some(1_000)), policy()).unwrap();
    core.observe("camera_clock", observation(1, 700, 10_000, 1))
        .unwrap();
    core.observe("camera_clock", observation(2, 800, 10_100, 2))
        .unwrap();
    core.observe("camera_clock", observation(3, 900, 10_200, 3))
        .unwrap();
    assert!(matches!(
        core.convert("camera_clock", 1_400),
        Err(ChronosError::InvalidObservation(_))
    ));
    assert_eq!(
        core.convert("camera_clock", 400),
        Err(ChronosError::TimestampOutsideModel)
    );
}

#[test]
/// Reported uncertainty cannot be finer than the declared clock resolution.
fn declared_resolution_bounds_model_uncertainty() {
    let mut core = ChronosCore::new();
    let mut coarse = descriptor(None);
    coarse.resolution_ns = 1_000_000;
    core.register(coarse, policy()).unwrap();
    for sequence in 1..=3 {
        core.observe(
            "camera_clock",
            observation(
                sequence,
                sequence as i64 * 1_000,
                sequence as i64 * 1_000,
                sequence,
            ),
        )
        .unwrap();
    }
    assert!(core.model("camera_clock").unwrap().uncertainty_ns() >= 1_000_000);
}

#[test]
/// Refitting a long observation window always covers its newest sample.
fn model_validity_always_contains_the_latest_observation() {
    let mut core = ChronosCore::new();
    let mut short = policy();
    short.observation_window = 10;
    short.tracking_timeout_ns = 10;
    short.holdover_duration_ns = 10;
    core.register(descriptor(None), short).unwrap();
    for sequence in 1..=6 {
        let timestamp = sequence as i64 * 1_000;
        core.observe(
            "camera_clock",
            observation(sequence, timestamp, timestamp, sequence),
        )
        .unwrap();
    }
    assert_eq!(
        core.convert("camera_clock", 6_000)
            .unwrap()
            .reference_timestamp_ns,
        6_000
    );
}

#[test]
/// Failed fitting does not commit its sample and is visible through FAULT.
fn fitting_failure_is_atomic_and_visible_as_fault() {
    let mut core = ChronosCore::new();
    let mut two_samples = policy();
    two_samples.minimum_observations = 2;
    core.register(descriptor(Some(i64::MAX as u64)), two_samples)
        .unwrap();
    core.observe("camera_clock", observation(1, i64::MAX - 100, 10_000, 1))
        .unwrap();
    core.observe("camera_clock", observation(2, 0, 10_100, 2))
        .unwrap();
    assert!(matches!(
        core.observe("camera_clock", observation(3, i64::MAX - 50, 10_150, 3),),
        Err(ChronosError::Overflow)
    ));
    let fault = core.status("camera_clock", 3).unwrap();
    assert_eq!(fault.state, DomainState::Fault);
    assert_eq!(fault.accepted_observations, 2);
    assert_eq!(fault.rejected_observations, 1);
    assert_eq!(
        core.convert("camera_clock", 0).unwrap().state,
        DomainState::Fault
    );

    core.update_reference_availability(false).unwrap();
    let unavailable = core.status("camera_clock", 3).unwrap();
    assert_eq!(unavailable.state, DomainState::Fault);
    assert!(unavailable.reason.contains("clock model processing failed"));
    assert_eq!(
        core.convert("camera_clock", 0).unwrap().uncertainty_ns,
        u64::MAX
    );
    core.update_reference_availability(true).unwrap();
    let recovered = core.status("camera_clock", 3).unwrap();
    assert_eq!(recovered.state, DomainState::Fault);
    assert_eq!(recovered.reason, "clock model processing failed");
}

#[test]
/// Step permits a new fit to correct an older model's forward bias without an artificial offset.
fn step_model_updates_allow_backward_correction() {
    let mut core = ChronosCore::new();
    let mut permissive = policy();
    permissive.minimum_observations = 3;
    permissive.observation_window = 3;
    permissive.max_drift_ppm = 1_000_000;
    permissive.max_residual_ns = 1_000;
    permissive.max_uncertainty_ns = 1_000_000;
    core.register(descriptor(None), permissive).unwrap();
    core.observe("camera_clock", observation(1, 0, 0, 1))
        .unwrap();
    core.observe("camera_clock", observation(2, 100, 100, 2))
        .unwrap();
    core.observe("camera_clock", observation(3, 200, 200, 3))
        .unwrap();
    let old_boundary = core
        .convert("camera_clock", 300)
        .unwrap()
        .reference_timestamp_ns;

    core.observe("camera_clock", observation(4, 300, 250, 4))
        .unwrap();
    let new_boundary = core
        .convert("camera_clock", 300)
        .unwrap()
        .reference_timestamp_ns;
    assert!(new_boundary < old_boundary);
    assert_eq!(new_boundary, 258);
}

/// Seed an identity model with permissive fitting limits and a selected update mode.
fn update_policy_core(mode: ModelUpdatePolicy, wrap: Option<u64>) -> ChronosCore {
    let mut core = ChronosCore::new();
    let mut limits = policy();
    limits.observation_window = 3;
    limits.max_drift_ppm = if wrap.is_some() { 500_000 } else { 1_000_000 };
    limits.model_update = mode;
    core.register(descriptor(wrap), limits).unwrap();
    for (sequence, device) in [(1, 0), (2, 100), (3, 200)] {
        core.observe(
            "camera_clock",
            observation(sequence, device, device, sequence * 100),
        )
        .unwrap();
    }
    core
}

#[test]
/// Both correction directions are bounded by elapsed steady time, including intervals too short to permit a one-nanosecond correction.
fn slew_limits_positive_and_negative_corrections() {
    for (reference, elapsed, expected) in [
        (250, 100, 290),
        (350, 100, 310),
        (250, 200, 280),
        (250, 1, 300),
        (250, 500, 258),
    ] {
        let mut core = update_policy_core(
            ModelUpdatePolicy::Slew {
                max_correction_ppm: 100_000,
            },
            None,
        );
        core.observe(
            "camera_clock",
            observation(4, 300, reference, 300 + elapsed),
        )
        .unwrap();
        let converted = core.convert("camera_clock", 300).unwrap();
        assert_eq!(converted.reference_timestamp_ns, expected);
        let mut raw = update_policy_core(ModelUpdatePolicy::Step {}, None);
        raw.observe(
            "camera_clock",
            observation(4, 300, reference, 300 + elapsed),
        )
        .unwrap();
        let fitted = raw.convert("camera_clock", 300).unwrap();
        assert_eq!(
            converted.uncertainty_ns,
            fitted.uncertainty_ns + expected.abs_diff(fitted.reference_timestamp_ns)
        );
    }
}

#[test]
/// Stable observations consume the remaining offset until the published model reaches the fit.
fn slew_converges_after_transient_samples_leave_window() {
    let mut core = update_policy_core(
        ModelUpdatePolicy::Slew {
            max_correction_ppm: 100_000,
        },
        None,
    );
    for sequence in 4..=20 {
        let device = (sequence as i64 - 1) * 100;
        core.observe(
            "camera_clock",
            observation(sequence, device, device - 50, sequence * 100),
        )
        .unwrap();
    }
    assert_eq!(
        core.convert("camera_clock", 1900)
            .unwrap()
            .reference_timestamp_ns,
        1850
    );
}

#[test]
/// A wrap uses the same normalized boundary as conversion when applying the correction limit.
fn slew_corrects_across_declared_wrap() {
    let mut core = update_policy_core(
        ModelUpdatePolicy::Slew {
            max_correction_ppm: 100_000,
        },
        Some(400),
    );
    core.observe("camera_clock", observation(4, 300, 300, 400))
        .unwrap();
    core.observe("camera_clock", observation(5, 0, 350, 500))
        .unwrap();
    assert_eq!(
        core.convert("camera_clock", 0)
            .unwrap()
            .reference_timestamp_ns,
        390
    );
}

#[test]
/// New device and reference generations start with the raw fit instead of smoothing old offsets.
fn slew_does_not_bridge_resets() {
    for reference_reset in [false, true] {
        let mut core = update_policy_core(
            ModelUpdatePolicy::Slew {
                max_correction_ppm: 1,
            },
            None,
        );
        if reference_reset {
            core.update_reference_generation(2).unwrap();
        } else {
            core.reset("camera_clock").unwrap();
        }
        for sequence in 1..=3 {
            let device = sequence as i64 * 100;
            core.observe(
                "camera_clock",
                observation(sequence, device, device + 500, 400 + sequence * 100),
            )
            .unwrap();
        }
        assert_eq!(
            core.convert("camera_clock", 300)
                .unwrap()
                .reference_timestamp_ns,
            800
        );
    }
}

#[test]
/// Remaining slew bias degrades quality until successive corrections bring it within policy.
fn slew_reports_unsynced_until_offset_recovers() {
    let mut core = ChronosCore::new();
    let mut limits = policy();
    limits.observation_window = 3;
    limits.max_drift_ppm = 1_000_000;
    limits.max_uncertainty_ns = 125;
    limits.model_update = ModelUpdatePolicy::Slew {
        max_correction_ppm: 100_000,
    };
    core.register(descriptor(None), limits).unwrap();
    for sequence in 1..=3 {
        let device = (sequence as i64 - 1) * 100;
        core.observe(
            "camera_clock",
            observation(sequence, device, device, sequence * 100),
        )
        .unwrap();
    }
    assert_eq!(
        core.status("camera_clock", 300).unwrap().state,
        DomainState::Tracking
    );
    core.observe("camera_clock", observation(4, 300, 250, 400))
        .unwrap();
    assert_eq!(
        core.status("camera_clock", 400).unwrap().state,
        DomainState::Unsynced
    );
    for sequence in 5..=20 {
        let device = (sequence as i64 - 1) * 100;
        core.observe(
            "camera_clock",
            observation(sequence, device, device - 50, sequence * 100),
        )
        .unwrap();
    }
    assert_eq!(
        core.status("camera_clock", 2000).unwrap().state,
        DomainState::Tracking
    );
}

#[test]
/// Missed counter cycles cannot silently corrupt the normalized timeline; explicit reset recovers.
fn wrapping_observation_gap_requires_reset() {
    let mut core = ChronosCore::new();
    core.register(descriptor(Some(1000)), policy()).unwrap();
    core.observe("camera_clock", observation(1, 800, 800, 1))
        .unwrap();
    let invalid = core
        .observe("camera_clock", observation(2, 850, 1850, 1001))
        .unwrap_err();
    assert!(invalid.to_string().contains("ambiguous"));
    assert_eq!(
        core.status("camera_clock", 1001)
            .unwrap()
            .accepted_observations,
        1
    );
    core.reset("camera_clock").unwrap();
    core.observe("camera_clock", observation(3, 850, 1850, 1002))
        .unwrap();
}
