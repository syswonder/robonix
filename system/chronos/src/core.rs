// SPDX-License-Identifier: MulanPSL-2.0

use std::collections::{HashMap, VecDeque};

use crate::model::{
    ChronosError, ClockDomainDescriptor, ClockDomainPolicy, ClockModel, ClockObservation,
    ConversionResult, DomainState, DomainStatus, ModelUpdatePolicy, PARTS_PER_BILLION,
    divide_rounded,
};

#[derive(Debug, Clone)]
struct Sample {
    device_timestamp_ns: i64,
    reference_timestamp_ns: i64,
    uncertainty_ns: u64,
}

#[derive(Debug, Clone)]
struct DomainRecord {
    descriptor: ClockDomainDescriptor,
    policy: ClockDomainPolicy,
    generation: u64,
    reference_generation: u64,
    model_version: u64,
    state: DomainState,
    reason: String,
    samples: VecDeque<Sample>,
    model: Option<ClockModel>,
    fitted_uncertainty_ns: u64,
    wrap_offset_ns: i128,
    last_raw_device_ns: Option<i64>,
    last_reference_ns: Option<i64>,
    last_seen_sequence_id: Option<u64>,
    last_seen_steady_ns: Option<u64>,
    last_observation_steady_ns: Option<u64>,
    accepted_observations: u64,
    rejected_observations: u64,
}

/// In-process registry and model estimator for device clock domains.
#[derive(Debug, Clone)]
pub struct ChronosCore {
    domains: HashMap<String, DomainRecord>,
    reference_generation: u64,
    reference_available: bool,
}

impl Default for ChronosCore {
    fn default() -> Self {
        Self {
            domains: HashMap::new(),
            reference_generation: 1,
            reference_available: true,
        }
    }
}

impl ChronosCore {
    /// Construct an empty core without reading any operating-system clock.
    pub fn new() -> Self {
        Self::default()
    }

    /// Return the generation of the reference clock used by all domain models.
    pub fn reference_generation(&self) -> u64 {
        self.reference_generation
    }

    /// Apply a reference generation and restart fitting for every device domain.
    ///
    /// Equal generations are idempotent. Older or zero generations are
    /// rejected. The last estimate remains available with its old reference
    /// identity while the reset is committed atomically across all domains.
    pub fn update_reference_generation(&mut self, generation: u64) -> Result<(), ChronosError> {
        if generation == 0 || generation < self.reference_generation {
            return Err(ChronosError::ReferenceGenerationOutOfOrder {
                current: self.reference_generation,
                received: generation,
            });
        }
        if generation == self.reference_generation {
            return Ok(());
        }
        let mut reset_domains = self.domains.clone();
        for record in reset_domains.values_mut() {
            begin_new_generation(record, true)?;
            record.reference_generation = generation;
            record.reason = "reference generation changed; retaining previous model".to_string();
        }
        self.domains = reset_domains;
        self.reference_generation = generation;
        Ok(())
    }

    /// Store reference availability as an orthogonal quality dimension.
    ///
    /// Domain state and diagnostics are retained across reference outages.
    /// Status and conversion queries overlay unavailable reference quality
    /// without mutating the fitted model or hiding an existing domain fault.
    pub fn update_reference_availability(&mut self, available: bool) -> Result<(), ChronosError> {
        if available == self.reference_available {
            return Ok(());
        }
        let mut updated = self.domains.clone();
        for record in updated.values_mut() {
            record.model_version = record
                .model_version
                .checked_add(1)
                .ok_or(ChronosError::Overflow)?;
            if let Some(model) = record.model.as_mut() {
                model.model_version = model
                    .model_version
                    .checked_add(1)
                    .ok_or(ChronosError::Overflow)?;
            }
        }
        self.domains = updated;
        self.reference_available = available;
        Ok(())
    }

    /// Register a domain after validating its static facts and quality policy.
    pub fn register(
        &mut self,
        descriptor: ClockDomainDescriptor,
        policy: ClockDomainPolicy,
    ) -> Result<DomainStatus, ChronosError> {
        descriptor.validate()?;
        policy.validate()?;
        if self.domains.contains_key(&descriptor.domain_id) {
            return Err(ChronosError::DuplicateDomain(descriptor.domain_id));
        }
        let domain_id = descriptor.domain_id.clone();
        let record = DomainRecord {
            descriptor,
            policy,
            generation: 1,
            reference_generation: self.reference_generation,
            model_version: 0,
            state: DomainState::Initializing,
            reason: "collecting observations".to_string(),
            samples: VecDeque::new(),
            model: None,
            fitted_uncertainty_ns: 0,
            wrap_offset_ns: 0,
            last_raw_device_ns: None,
            last_reference_ns: None,
            last_seen_sequence_id: None,
            last_seen_steady_ns: None,
            last_observation_steady_ns: None,
            accepted_observations: 0,
            rejected_observations: 0,
        };
        self.domains.insert(domain_id.clone(), record);
        self.status(&domain_id, 0)
    }

    /// Remove a domain and all models and observations associated with it.
    pub fn unregister(&mut self, domain_id: &str) -> Result<(), ChronosError> {
        self.domains
            .remove(domain_id)
            .map(|_| ())
            .ok_or_else(|| ChronosError::UnknownDomain(domain_id.to_string()))
    }

    /// Start a new generation after a confirmed device restart or reference change.
    pub fn reset(&mut self, domain_id: &str) -> Result<DomainStatus, ChronosError> {
        let record = self
            .domains
            .get_mut(domain_id)
            .ok_or_else(|| ChronosError::UnknownDomain(domain_id.to_string()))?;
        begin_new_generation(record, false)?;
        Ok(status_from_record(
            domain_id,
            record,
            0,
            self.reference_available,
        ))
    }

    /// Validate and incorporate one paired observation into a domain model.
    ///
    /// Device or reference time regression starts a new generation unless the
    /// device regression matches the descriptor's declared counter wrap.
    pub fn observe(
        &mut self,
        domain_id: &str,
        observation: ClockObservation,
    ) -> Result<DomainStatus, ChronosError> {
        if !self.reference_available {
            return Err(ChronosError::ReferenceUnavailable);
        }
        let record = self
            .domains
            .get_mut(domain_id)
            .ok_or_else(|| ChronosError::UnknownDomain(domain_id.to_string()))?;
        if let Err(error) = validate_observation_order(record, &observation) {
            record.rejected_observations = record.rejected_observations.saturating_add(1);
            return Err(error);
        }
        if let Err(error) = validate_observation_value(record, &observation) {
            record.rejected_observations = record.rejected_observations.saturating_add(1);
            record.last_seen_sequence_id = Some(observation.sequence_id);
            record.last_seen_steady_ns = Some(observation.steady_received_ns);
            return Err(error);
        }
        let reset = observation_requires_reset(record, &observation);
        let mut candidate = record.clone();
        if reset {
            begin_new_generation(&mut candidate, false)?;
        }
        let normalized_device =
            match normalize_observation(&mut candidate, observation.device_timestamp_ns) {
                Ok(value) => value,
                Err(error) => {
                    mark_fault(record, &observation);
                    return Err(error);
                }
            };
        let outlier = match is_outlier(&candidate, normalized_device, &observation) {
            Ok(value) => value,
            Err(error) => {
                mark_fault(record, &observation);
                return Err(error);
            }
        };
        if outlier {
            let predicted = match candidate
                .model
                .as_ref()
                .expect("outlier checks require a model")
                .extrapolate_normalized(normalized_device)
            {
                Ok(value) => value,
                Err(error) => {
                    mark_fault(record, &observation);
                    return Err(error);
                }
            };
            let residual_ns =
                match absolute_difference(predicted, observation.reference_timestamp_ns) {
                    Ok(value) => value,
                    Err(error) => {
                        mark_fault(record, &observation);
                        return Err(error);
                    }
                };
            record.rejected_observations = record.rejected_observations.saturating_add(1);
            record.last_seen_sequence_id = Some(observation.sequence_id);
            record.last_seen_steady_ns = Some(observation.steady_received_ns);
            let limit_ns = outlier_limit(&candidate, observation.uncertainty_ns);
            return Err(ChronosError::ObservationOutlier {
                residual_ns,
                limit_ns,
            });
        }
        if let Err(error) =
            accept_observation(&mut candidate, normalized_device, observation.clone())
        {
            mark_fault(record, &observation);
            return Err(error);
        }
        if let Err(error) = update_model(&mut candidate) {
            mark_fault(record, &observation);
            return Err(error);
        }
        *record = candidate;
        Ok(status_from_record(
            domain_id,
            record,
            record.last_observation_steady_ns.unwrap_or(0),
            self.reference_available,
        ))
    }

    /// Return an owned model with current reference quality applied.
    pub fn model(&self, domain_id: &str) -> Result<ClockModel, ChronosError> {
        let mut model = self.stored_model(domain_id)?.clone();
        if !self.reference_available {
            if model.state != DomainState::Fault {
                model.state = DomainState::Unsynced;
            }
            model.uncertainty_ns = u64::MAX;
        }
        Ok(model)
    }

    /// Borrow coefficients without applying orthogonal reference quality.
    fn stored_model(&self, domain_id: &str) -> Result<&ClockModel, ChronosError> {
        let record = self
            .domains
            .get(domain_id)
            .ok_or_else(|| ChronosError::UnknownDomain(domain_id.to_string()))?;
        record
            .model
            .as_ref()
            .ok_or(ChronosError::ModelUnavailable(record.state))
    }

    /// Convert a timestamp with the latest model and return its exact identity.
    pub fn convert(
        &self,
        domain_id: &str,
        device_timestamp_ns: i64,
    ) -> Result<ConversionResult, ChronosError> {
        let mut result = self.stored_model(domain_id)?.convert(device_timestamp_ns)?;
        if !self.reference_available {
            if result.state != DomainState::Fault {
                result.state = DomainState::Unsynced;
            }
            result.uncertainty_ns = u64::MAX;
        }
        Ok(result)
    }

    /// Return a domain generation for binding queued observations to resets.
    pub(crate) fn domain_generation(&self, domain_id: &str) -> Result<u64, ChronosError> {
        self.domains
            .get(domain_id)
            .map(|record| record.generation)
            .ok_or_else(|| ChronosError::UnknownDomain(domain_id.to_string()))
    }

    /// Return current status with age measured only against monotonic input.
    pub fn status(
        &self,
        domain_id: &str,
        now_steady_ns: u64,
    ) -> Result<DomainStatus, ChronosError> {
        let record = self
            .domains
            .get(domain_id)
            .ok_or_else(|| ChronosError::UnknownDomain(domain_id.to_string()))?;
        if record
            .last_seen_steady_ns
            .is_some_and(|last| now_steady_ns < last)
        {
            return Err(ChronosError::ObservationOutOfOrder);
        }
        Ok(status_from_record(
            domain_id,
            record,
            now_steady_ns,
            self.reference_available,
        ))
    }

    /// Advance mature domains through holdover and unsynchronized states.
    ///
    /// Uncertainty grows from the fitted value using the configured drift
    /// bound. Each externally visible change produces a new model version.
    pub fn refresh(&mut self, now_steady_ns: u64) -> Result<(), ChronosError> {
        if self
            .domains
            .values()
            .filter_map(|record| record.last_seen_steady_ns)
            .any(|last| now_steady_ns < last)
        {
            return Err(ChronosError::ObservationOutOfOrder);
        }
        let mut refreshed = self.domains.clone();
        for record in refreshed.values_mut() {
            refresh_record(record, now_steady_ns)?;
        }
        self.domains = refreshed;
        Ok(())
    }
}

/// Require strictly increasing sequence and monotonic receipt values.
fn validate_observation_order(
    record: &DomainRecord,
    observation: &ClockObservation,
) -> Result<(), ChronosError> {
    let sequence_is_old = record
        .last_seen_sequence_id
        .is_some_and(|sequence| observation.sequence_id <= sequence);
    let steady_is_old = record
        .last_seen_steady_ns
        .is_some_and(|steady| observation.steady_received_ns <= steady);
    if sequence_is_old || steady_is_old {
        return Err(ChronosError::ObservationOutOfOrder);
    }
    Ok(())
}

/// Distinguish declared counter wrap from a reset or reference regression.
fn observation_requires_reset(record: &DomainRecord, observation: &ClockObservation) -> bool {
    let device_reset = record.last_raw_device_ns.is_some_and(|last| {
        if observation.device_timestamp_ns >= last {
            return false;
        }
        let regression = i128::from(last) - i128::from(observation.device_timestamp_ns);
        record
            .descriptor
            .wrap_period_ns
            .is_none_or(|period| regression <= i128::from(period) / 2)
    });
    let reference_reset = record
        .last_reference_ns
        .is_some_and(|last| observation.reference_timestamp_ns < last);
    device_reset || reference_reset
}

/// Clear generation-specific history while optionally retaining a stale model.
fn begin_new_generation(record: &mut DomainRecord, retain_model: bool) -> Result<(), ChronosError> {
    record.generation = record
        .generation
        .checked_add(1)
        .ok_or(ChronosError::Overflow)?;
    if retain_model {
        if let Some(model) = record.model.as_mut() {
            model.model_version = model
                .model_version
                .checked_add(1)
                .ok_or(ChronosError::Overflow)?;
            model.state = DomainState::Unsynced;
        }
    } else {
        record.model = None;
    }
    record.model_version = 0;
    record.state = DomainState::Reset;
    record.reason = "clock reset or time regression".to_string();
    record.samples.clear();
    record.fitted_uncertainty_ns = 0;
    record.wrap_offset_ns = 0;
    record.last_raw_device_ns = None;
    record.last_reference_ns = None;
    record.last_seen_sequence_id = None;
    record.last_observation_steady_ns = None;
    Ok(())
}

/// Require wrapping counters to use their declared canonical raw range.
fn validate_observation_value(
    record: &DomainRecord,
    observation: &ClockObservation,
) -> Result<(), ChronosError> {
    if let Some(period) = record.descriptor.wrap_period_ns
        && (observation.device_timestamp_ns < 0 || observation.device_timestamp_ns >= period as i64)
    {
        return Err(ChronosError::InvalidObservation(
            "device timestamp is outside the declared wrap period".to_string(),
        ));
    }
    if let (Some(period), Some(previous_steady)) = (
        record.descriptor.wrap_period_ns,
        record.last_observation_steady_ns,
    ) {
        let elapsed = observation.steady_received_ns - previous_steady;
        let maximum_rate = 1_000_000 + u128::from(record.policy.max_drift_ppm);
        let ambiguous_after = (u128::from(period) * 1_000_000).div_ceil(maximum_rate * 2);
        if u128::from(elapsed) >= ambiguous_after {
            return Err(ChronosError::InvalidObservation(
                "counter wrap is ambiguous after observation gap; reset the domain".into(),
            ));
        }
    }
    if record
        .last_raw_device_ns
        .is_some_and(|last| observation.device_timestamp_ns == last)
    {
        return Err(ChronosError::InvalidObservation(
            "device timestamp repeats the previous observation".to_string(),
        ));
    }
    Ok(())
}

/// Extend a declared wrapping counter into a generation-local signed timeline.
fn normalize_observation(record: &mut DomainRecord, raw: i64) -> Result<i64, ChronosError> {
    if let (Some(last), Some(period)) =
        (record.last_raw_device_ns, record.descriptor.wrap_period_ns)
        && raw < last
        && i128::from(last) - i128::from(raw) > i128::from(period) / 2
    {
        record.wrap_offset_ns = record
            .wrap_offset_ns
            .checked_add(i128::from(period))
            .ok_or(ChronosError::Overflow)?;
    }
    let normalized = i128::from(raw)
        .checked_add(record.wrap_offset_ns)
        .ok_or(ChronosError::Overflow)?;
    i64::try_from(normalized).map_err(|_| ChronosError::Overflow)
}

/// Reject only clear residual violations from an already usable model.
fn is_outlier(
    record: &DomainRecord,
    normalized_device_ns: i64,
    observation: &ClockObservation,
) -> Result<bool, ChronosError> {
    if !matches!(record.state, DomainState::Tracking | DomainState::Holdover) {
        return Ok(false);
    }
    let Some(model) = record.model.as_ref() else {
        return Ok(false);
    };
    let predicted = model.extrapolate_normalized(normalized_device_ns)?;
    let residual = absolute_difference(predicted, observation.reference_timestamp_ns)?;
    Ok(residual > outlier_limit(record, observation.uncertainty_ns))
}

/// Combine the configured residual bound with both measurement uncertainties.
fn outlier_limit(record: &DomainRecord, observation_uncertainty_ns: u64) -> u64 {
    record
        .policy
        .max_residual_ns
        .saturating_add(record.fitted_uncertainty_ns)
        .saturating_add(observation_uncertainty_ns)
}

/// Store one accepted sample and its ordering boundary before fitting.
fn accept_observation(
    record: &mut DomainRecord,
    normalized_device_ns: i64,
    observation: ClockObservation,
) -> Result<(), ChronosError> {
    record.samples.push_back(Sample {
        device_timestamp_ns: normalized_device_ns,
        reference_timestamp_ns: observation.reference_timestamp_ns,
        uncertainty_ns: observation.uncertainty_ns,
    });
    while record.samples.len() > record.policy.observation_window {
        record.samples.pop_front();
    }
    record.last_raw_device_ns = Some(observation.device_timestamp_ns);
    record.last_reference_ns = Some(observation.reference_timestamp_ns);
    record.last_seen_sequence_id = Some(observation.sequence_id);
    record.last_seen_steady_ns = Some(observation.steady_received_ns);
    record.last_observation_steady_ns = Some(observation.steady_received_ns);
    record.accepted_observations = record
        .accepted_observations
        .checked_add(1)
        .ok_or(ChronosError::Overflow)?;
    Ok(())
}

/// Make an internal processing failure visible without committing staged samples.
fn mark_fault(record: &mut DomainRecord, observation: &ClockObservation) {
    record.state = DomainState::Fault;
    record.reason = "clock model processing failed".to_string();
    record.model_version = record.model_version.saturating_add(1);
    record.rejected_observations = record.rejected_observations.saturating_add(1);
    record.last_seen_sequence_id = Some(observation.sequence_id);
    record.last_seen_steady_ns = Some(observation.steady_received_ns);
    if let Some(model) = record.model.as_mut() {
        model.state = DomainState::Fault;
        model.model_version = if model.generation == record.generation
            && model.reference_generation == record.reference_generation
        {
            record.model_version
        } else {
            model.model_version.saturating_add(1)
        };
    }
}

/// Fit or update an affine model once the domain has enough observations.
fn update_model(record: &mut DomainRecord) -> Result<(), ChronosError> {
    if record.samples.len() < record.policy.minimum_observations {
        if record.state != DomainState::Reset {
            record.state = DomainState::Initializing;
            record.reason = "collecting observations".to_string();
        }
        return Ok(());
    }
    let mut fit = fit_samples(&record.samples)?;
    fit.uncertainty_ns = fit
        .uncertainty_ns
        .saturating_add(record.descriptor.resolution_ns);
    let drift_limit_ppb = i128::from(record.policy.max_drift_ppm)
        .checked_mul(1_000)
        .ok_or(ChronosError::Overflow)?;
    let next_version = record
        .model_version
        .checked_add(1)
        .ok_or(ChronosError::Overflow)?;
    let mut model = build_model(record, &fit, next_version, DomainState::Tracking)?;
    apply_model_update_policy(
        record.policy.model_update,
        record.model.as_ref(),
        &mut model,
        &mut fit,
    )?;
    let quality_ok = i128::from(fit.rate_adjust_ppb).abs() <= drift_limit_ppb
        && fit.uncertainty_ns <= record.policy.max_uncertainty_ns;
    let state = if quality_ok {
        DomainState::Tracking
    } else {
        DomainState::Unsynced
    };
    model.state = state;
    model.uncertainty_ns = fit.uncertainty_ns;
    record.state = state;
    record.reason = if quality_ok {
        "model satisfies clock domain policy".to_string()
    } else {
        "drift or uncertainty exceeds clock domain policy".to_string()
    };
    record.model_version = next_version;
    record.fitted_uncertainty_ns = fit.uncertainty_ns;
    record.model = Some(model);
    Ok(())
}

/// Bound the phase correction at the latest sample using elapsed observation steady time.
/// Step uses the fit directly; slew includes its remaining offset in reported uncertainty.
fn apply_model_update_policy(
    policy: ModelUpdatePolicy,
    previous: Option<&ClockModel>,
    next: &mut ClockModel,
    fit: &mut Fit,
) -> Result<(), ChronosError> {
    let ModelUpdatePolicy::Slew { max_correction_ppm } = policy else {
        return Ok(());
    };
    let Some(previous) = previous.filter(|model| {
        model.generation == next.generation
            && model.reference_generation == next.reference_generation
    }) else {
        return Ok(());
    };
    let boundary = i128::from(next.latest_raw_device_ns)
        .checked_add(i128::from(next.wrap_offset_ns))
        .and_then(|value| i64::try_from(value).ok())
        .ok_or(ChronosError::Overflow)?;
    let old_reference = previous.extrapolate_normalized(boundary)?;
    let new_reference = next.extrapolate_normalized(boundary)?;
    let elapsed = next
        .last_observation_steady_ns
        .checked_sub(previous.last_observation_steady_ns)
        .ok_or(ChronosError::ObservationOutOfOrder)?;
    let budget = i128::from(elapsed)
        .checked_mul(i128::from(max_correction_ppm))
        .ok_or(ChronosError::Overflow)?
        / 1_000_000;
    let requested = i128::from(new_reference) - i128::from(old_reference);
    let correction = requested.clamp(-budget, budget) - requested;
    if correction != 0 {
        let correction = i64::try_from(correction).map_err(|_| ChronosError::Overflow)?;
        next.residual_offset_ns = next
            .residual_offset_ns
            .checked_add(correction)
            .ok_or(ChronosError::Overflow)?;
        fit.residual_offset_ns = next.residual_offset_ns;
        fit.uncertainty_ns = fit.uncertainty_ns.saturating_add(correction.unsigned_abs());
    }
    Ok(())
}

#[derive(Debug)]
struct Fit {
    device_anchor_ns: i64,
    reference_anchor_ns: i64,
    rate_adjust_ppb: i64,
    residual_offset_ns: i64,
    uncertainty_ns: u64,
}

/// Fit centered least squares without converting epoch-sized values to floats.
fn fit_samples(samples: &VecDeque<Sample>) -> Result<Fit, ChronosError> {
    let first = samples.front().expect("fitting requires observations");
    let count = samples.len() as f64;
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    for sample in samples {
        sum_x +=
            (i128::from(sample.device_timestamp_ns) - i128::from(first.device_timestamp_ns)) as f64;
        sum_y += (i128::from(sample.reference_timestamp_ns)
            - i128::from(first.reference_timestamp_ns)) as f64;
    }
    let mean_x = sum_x / count;
    let mean_y = sum_y / count;
    let mut covariance = 0.0;
    let mut variance = 0.0;
    for sample in samples {
        let x =
            (i128::from(sample.device_timestamp_ns) - i128::from(first.device_timestamp_ns)) as f64;
        let y = (i128::from(sample.reference_timestamp_ns)
            - i128::from(first.reference_timestamp_ns)) as f64;
        covariance += (x - mean_x) * (y - mean_y);
        variance += (x - mean_x) * (x - mean_x);
    }
    if variance == 0.0 {
        return Err(ChronosError::InvalidObservation(
            "device observations do not advance".to_string(),
        ));
    }
    finish_fit(samples, first, covariance / variance)
}

/// Convert the floating-point slope into a centered integer clock model.
fn finish_fit(samples: &VecDeque<Sample>, first: &Sample, slope: f64) -> Result<Fit, ChronosError> {
    if !slope.is_finite() {
        return Err(ChronosError::Overflow);
    }
    let rate = ((slope - 1.0) * PARTS_PER_BILLION as f64).round();
    if !(i64::MIN as f64..=i64::MAX as f64).contains(&rate) {
        return Err(ChronosError::Overflow);
    }
    let rate_adjust_ppb = rate as i64;
    let mut residual_sum = 0_i128;
    for sample in samples {
        let delta = i128::from(sample.device_timestamp_ns)
            .checked_sub(i128::from(first.device_timestamp_ns))
            .ok_or(ChronosError::Overflow)?;
        let predicted = predict_from_first(first, delta, rate_adjust_ppb, 0)?;
        residual_sum = residual_sum
            .checked_add(i128::from(sample.reference_timestamp_ns) - i128::from(predicted))
            .ok_or(ChronosError::Overflow)?;
    }
    let residual_offset = divide_rounded(residual_sum, samples.len() as i128);
    let residual_offset_ns = i64::try_from(residual_offset).map_err(|_| ChronosError::Overflow)?;
    let mut residual_square_sum = 0.0;
    let mut input_uncertainty = 0;
    for sample in samples {
        let delta = i128::from(sample.device_timestamp_ns)
            .checked_sub(i128::from(first.device_timestamp_ns))
            .ok_or(ChronosError::Overflow)?;
        let predicted = predict_from_first(first, delta, rate_adjust_ppb, residual_offset_ns)?;
        let residual = absolute_difference(predicted, sample.reference_timestamp_ns)?;
        residual_square_sum += (residual as f64).powi(2);
        input_uncertainty = input_uncertainty.max(sample.uncertainty_ns);
    }
    let rms = (residual_square_sum / samples.len() as f64).sqrt().ceil() as u64;
    Ok(Fit {
        device_anchor_ns: first.device_timestamp_ns,
        reference_anchor_ns: first.reference_timestamp_ns,
        rate_adjust_ppb,
        residual_offset_ns,
        uncertainty_ns: input_uncertainty.saturating_add(rms),
    })
}

/// Evaluate a fitted rate from the first paired observation using integers.
fn predict_from_first(
    first: &Sample,
    delta: i128,
    rate_adjust_ppb: i64,
    residual_offset_ns: i64,
) -> Result<i64, ChronosError> {
    let rate = divide_rounded(
        delta
            .checked_mul(i128::from(rate_adjust_ppb))
            .ok_or(ChronosError::Overflow)?,
        PARTS_PER_BILLION,
    );
    let predicted = i128::from(first.reference_timestamp_ns)
        .checked_add(delta)
        .and_then(|value| value.checked_add(rate))
        .and_then(|value| value.checked_add(i128::from(residual_offset_ns)))
        .ok_or(ChronosError::Overflow)?;
    i64::try_from(predicted).map_err(|_| ChronosError::Overflow)
}

/// Materialize a fitted model with a bounded future validity interval.
fn build_model(
    record: &DomainRecord,
    fit: &Fit,
    model_version: u64,
    state: DomainState,
) -> Result<ClockModel, ChronosError> {
    let first = record.samples.front().expect("model requires observations");
    let latest = record.samples.back().expect("model requires observations");
    let validity_extension = record
        .policy
        .tracking_timeout_ns
        .saturating_add(record.policy.holdover_duration_ns);
    let (valid_from, extension) = if let Some(period) = record.descriptor.wrap_period_ns {
        let half_period = (period / 2).max(1);
        let unambiguous_margin = half_period.saturating_sub(1);
        (
            i128::from(latest.device_timestamp_ns) - i128::from(unambiguous_margin),
            validity_extension.min(unambiguous_margin),
        )
    } else {
        (i128::from(first.device_timestamp_ns), validity_extension)
    };
    let valid_from = valid_from.max(i128::from(first.device_timestamp_ns));
    let valid_until = i128::from(latest.device_timestamp_ns)
        .saturating_add(i128::from(extension))
        .min(i128::from(i64::MAX));
    Ok(ClockModel {
        domain_id: record.descriptor.domain_id.clone(),
        generation: record.generation,
        reference_generation: record.reference_generation,
        model_version,
        device_anchor_ns: fit.device_anchor_ns,
        reference_anchor_ns: fit.reference_anchor_ns,
        rate_adjust_ppb: fit.rate_adjust_ppb,
        residual_offset_ns: fit.residual_offset_ns,
        uncertainty_ns: fit.uncertainty_ns,
        valid_from_device_ns: i64::try_from(valid_from).map_err(|_| ChronosError::Overflow)?,
        valid_until_device_ns: i64::try_from(valid_until).map_err(|_| ChronosError::Overflow)?,
        sample_count: record.samples.len() as u64,
        state,
        last_observation_steady_ns: record.last_observation_steady_ns.unwrap_or(0),
        latest_raw_device_ns: record.last_raw_device_ns.unwrap_or(fit.device_anchor_ns),
        wrap_period_ns: record.descriptor.wrap_period_ns,
        wrap_offset_ns: i64::try_from(record.wrap_offset_ns).map_err(|_| ChronosError::Overflow)?,
    })
}

/// Apply freshness and holdover limits to one mature domain.
fn refresh_record(record: &mut DomainRecord, now_steady_ns: u64) -> Result<(), ChronosError> {
    if !matches!(
        record.state,
        DomainState::Tracking | DomainState::Holdover | DomainState::Unsynced | DomainState::Fault
    ) {
        return Ok(());
    }
    let Some(last) = record.last_observation_steady_ns else {
        return Ok(());
    };
    let age = now_steady_ns - last;
    if age <= record.policy.tracking_timeout_ns {
        return Ok(());
    }
    let holdover_age = age - record.policy.tracking_timeout_ns;
    let drift_product = u128::from(holdover_age) * u128::from(record.policy.max_drift_ppm);
    let drift_growth = drift_product / 1_000_000 + u128::from(drift_product % 1_000_000 != 0);
    let drift_growth = u64::try_from(drift_growth).unwrap_or(u64::MAX);
    let uncertainty = record.fitted_uncertainty_ns.saturating_add(drift_growth);
    let next_state = match record.state {
        DomainState::Fault => DomainState::Fault,
        DomainState::Unsynced => DomainState::Unsynced,
        _ if holdover_age <= record.policy.holdover_duration_ns
            && uncertainty <= record.policy.max_uncertainty_ns =>
        {
            DomainState::Holdover
        }
        _ => DomainState::Unsynced,
    };
    if record.state != next_state
        || record
            .model
            .as_ref()
            .is_some_and(|model| model.uncertainty_ns != uncertainty)
    {
        record.model_version = record
            .model_version
            .checked_add(1)
            .ok_or(ChronosError::Overflow)?;
        record.state = next_state;
        record.reason = match next_state {
            DomainState::Holdover => "observations are stale; using bounded holdover".to_string(),
            DomainState::Unsynced => {
                "observations are stale; estimate uncertainty is unbounded by policy".to_string()
            }
            DomainState::Fault => "clock model processing failed; retaining estimate".to_string(),
            _ => unreachable!("freshness refresh only produces degraded states"),
        };
        if let Some(model) = record.model.as_mut() {
            model.model_version = record.model_version;
            model.state = next_state;
            model.uncertainty_ns = uncertainty;
        }
    }
    Ok(())
}

/// Build a status snapshot without changing model or state.
fn status_from_record(
    domain_id: &str,
    record: &DomainRecord,
    now_steady_ns: u64,
    reference_available: bool,
) -> DomainStatus {
    let mut status = DomainStatus {
        domain_id: domain_id.to_string(),
        generation: record.generation,
        reference_generation: record.reference_generation,
        model_version: record.model_version,
        state: record.state,
        accepted_observations: record.accepted_observations,
        rejected_observations: record.rejected_observations,
        last_observation_age_ns: record
            .last_observation_steady_ns
            .map(|last| now_steady_ns.saturating_sub(last)),
        uncertainty_ns: record
            .model
            .as_ref()
            .filter(|model| {
                model.generation == record.generation
                    && model.reference_generation == record.reference_generation
            })
            .map(|model| model.uncertainty_ns),
        reason: record.reason.clone(),
    };
    if !reference_available {
        if status.state != DomainState::Fault {
            status.state = DomainState::Unsynced;
        }
        status.uncertainty_ns = status.uncertainty_ns.map(|_| u64::MAX);
        status.reason = format!(
            "reference clock is unavailable; domain status: {}",
            record.reason
        );
    }
    status
}

/// Compute an absolute signed timestamp difference without overflowing i64.
fn absolute_difference(left: i64, right: i64) -> Result<u64, ChronosError> {
    let difference = (i128::from(left) - i128::from(right)).abs();
    u64::try_from(difference).map_err(|_| ChronosError::Overflow)
}
