// SPDX-License-Identifier: MulanPSL-2.0

use std::collections::BTreeMap;
use std::fmt::Debug;
use std::sync::Arc;

use crate::config::{
    ChronosConfig, ChronosConfigError, CoreConfig, DomainStrategyConfig, ObservationConfig,
    ObservationReferenceKind, TimestampScaleKind,
};
use crate::core::ChronosCore;
use crate::model::{
    ChronosError, ClockModel, ClockObservation, ConversionResult, DomainState, DomainStatus,
};
use crate::reference::ReferenceError;
use crate::snapshot::{ClockModelSnapshot, ClockModelsSnapshot, ClockQualitySnapshot};
use crate::virtual_clock::{
    VirtualClock, VirtualClockSnapshot, VirtualReferenceTime, VirtualSteadyTime,
};
use thiserror::Error;

/// Quality derived from one configured direct timestamp relationship.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DirectDomainQuality {
    pub domain_id: String,
    pub time_scale: TimestampScaleKind,
    pub reference_generation: u64,
    pub reference_available: bool,
    pub reference_verified: bool,
    pub uncertainty_ns: u64,
    pub reason: String,
}

/// Quality reported for either configured timestamp-domain strategy.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DomainQualityReport {
    Direct(DirectDomainQuality),
    AffineModel(DomainStatus),
}

impl DomainQualityReport {
    /// Return the configured identifier shared by both strategy reports.
    pub fn domain_id(&self) -> &str {
        match self {
            Self::Direct(report) => &report.domain_id,
            Self::AffineModel(report) => &report.domain_id,
        }
    }
}

/// One coherent reference and device-domain quality snapshot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClockQualityReport {
    pub reference: VirtualReferenceTime,
    pub steady: VirtualSteadyTime,
    pub domains: Vec<DomainQualityReport>,
}

/// One continuing refresh failure observed by the quality runtime.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualityRefreshFailure {
    pub message: String,
    pub consecutive_failures: u64,
    pub first_observed_steady: Option<VirtualSteadyTime>,
    pub last_observed_steady: Option<VirtualSteadyTime>,
}

/// Latest report and refresh health produced without stopping on errors.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualityRuntimeStatus {
    pub report: Option<ClockQualityReport>,
    pub last_successful_refresh_steady: Option<VirtualSteadyTime>,
    pub refresh_failure: Option<QualityRefreshFailure>,
}

impl QualityRuntimeStatus {
    /// Return whether the retained report predates the current refresh failure.
    pub fn report_is_historical(&self) -> bool {
        self.report.is_some() && self.refresh_failure.is_some()
    }
}

/// Runtime-issued clock pair captured at message ingress.
#[derive(Debug, Clone)]
pub struct ObservationSnapshot {
    owner: Arc<()>,
    clock: VirtualClockSnapshot,
    domain_generations: BTreeMap<String, u64>,
}

impl ObservationSnapshot {
    /// Return the captured reference-time evidence.
    pub fn reference(&self) -> &VirtualReferenceTime {
        &self.clock.reference
    }

    /// Return the captured steady receive time.
    pub fn steady(&self) -> VirtualSteadyTime {
        self.clock.steady
    }
}

/// Errors produced while composing clocks, configuration, and model updates.
#[derive(Debug, Error)]
pub enum ChronosRuntimeError {
    #[error(transparent)]
    Config(#[from] ChronosConfigError),
    #[error(transparent)]
    Reference(#[from] ReferenceError),
    #[error(transparent)]
    Core(#[from] ChronosError),
    #[error("unknown configured clock domain '{0}'")]
    UnknownDomain(String),
    #[error("clock domain '{0}' uses direct timestamps and has no affine model")]
    DirectDomain(String),
    #[error("configured reference-time adjustment overflows the timestamp range")]
    TimestampAdjustmentOverflow,
    #[error(
        "clock domain '{domain_id}' observation interval has not elapsed; {remaining_ns} ns remain"
    )]
    ObservationIntervalNotElapsed {
        domain_id: String,
        remaining_ns: u64,
    },
    #[error("observation snapshot belongs to another Chronos instance")]
    ForeignObservationSnapshot,
    #[error(
        "observation snapshot reference generation {snapshot} does not match current generation {current}"
    )]
    ObservationReferenceGenerationMismatch { current: u64, snapshot: u64 },
    #[error(
        "observation snapshot domain generation {snapshot} for '{domain_id}' does not match current generation {current}"
    )]
    ObservationDomainGenerationMismatch {
        domain_id: String,
        current: u64,
        snapshot: u64,
    },
    #[error("failed to start Chronos quality worker: {0}")]
    QualityWorkerStart(#[source] std::io::Error),
    #[error("Chronos quality worker panicked")]
    QualityWorkerPanicked,
    #[error("Chronos runtime state is unavailable after a worker panic")]
    QualityStatePoisoned,
}

/// Clock reads required by the composition layer.
trait ClockSource: Debug + Send {
    fn snapshot(&mut self) -> Result<VirtualClockSnapshot, ReferenceError>;
    fn steady_time(&self) -> Result<VirtualSteadyTime, ReferenceError>;
}

impl ClockSource for VirtualClock {
    fn snapshot(&mut self) -> Result<VirtualClockSnapshot, ReferenceError> {
        self.snapshot()
    }

    fn steady_time(&self) -> Result<VirtualSteadyTime, ReferenceError> {
        self.steady_time()
    }
}

/// In-process Chronos API that preserves ordering across its pure components.
#[derive(Debug)]
pub struct Chronos {
    config: CoreConfig,
    clock: Box<dyn ClockSource>,
    core: ChronosCore,
    observation_owner: Arc<()>,
    last_observation_sample_ns: BTreeMap<String, u64>,
    latest_quality: Option<ClockQualityReport>,
    last_successful_refresh_steady: Option<VirtualSteadyTime>,
    last_reference_change_steady: Option<VirtualSteadyTime>,
    refresh_failure: Option<QualityRefreshFailure>,
}

impl Chronos {
    /// Validate configuration, construct clocks, and register modeled domains.
    pub fn new(config: ChronosConfig) -> Result<Self, ChronosRuntimeError> {
        let core_config = config.chronos.core;
        let clock = VirtualClock::new(core_config.reference.clone())?;
        Self::with_clock(core_config, Box::new(clock))
    }

    /// Register modeled domains around one clock implementation.
    fn with_clock(
        core_config: CoreConfig,
        clock: Box<dyn ClockSource>,
    ) -> Result<Self, ChronosRuntimeError> {
        let registrations = core_config.affine_model_domains()?;
        let mut core = ChronosCore::new();
        for (descriptor, policy) in registrations {
            core.register(descriptor, policy)?;
        }
        Ok(Self {
            config: core_config,
            clock,
            core,
            observation_owner: Arc::new(()),
            last_observation_sample_ns: BTreeMap::new(),
            latest_quality: None,
            last_successful_refresh_steady: None,
            last_reference_change_steady: None,
            refresh_failure: None,
        })
    }

    /// Return the validated configuration that defines timestamp semantics.
    pub fn config(&self) -> &CoreConfig {
        &self.config
    }

    /// Read steady time independently of reference-clock availability.
    pub fn steady_time(&self) -> Result<VirtualSteadyTime, ChronosRuntimeError> {
        Ok(self.clock.steady_time()?)
    }

    /// Read virtual reference time and refresh the coherent quality report.
    pub fn reference_time(&mut self) -> Result<VirtualReferenceTime, ChronosRuntimeError> {
        Ok(self.refresh()?.reference.clone())
    }

    /// Sample reference and steady time together for an ingress observation.
    ///
    /// Local adapters can take this snapshot as soon as a message arrives and
    /// later pass it to `observe_device_timestamp_at`, avoiding queue and
    /// scheduling delay in the fitted relationship.
    pub fn snapshot(&mut self) -> Result<ObservationSnapshot, ChronosRuntimeError> {
        let clock = self.sample_and_apply()?;
        let domain_generations = self
            .config
            .domains
            .iter()
            .filter_map(|(domain_id, domain)| {
                matches!(domain.strategy, DomainStrategyConfig::AffineModel { .. })
                    .then_some(domain_id)
            })
            .map(|domain_id| Ok((domain_id.clone(), self.core.domain_generation(domain_id)?)))
            .collect::<Result<_, ChronosError>>()?;
        Ok(ObservationSnapshot {
            owner: self.observation_owner.clone(),
            clock,
            domain_generations,
        })
    }

    /// Admit a clock pair from the owned same-host helper without replacing its receive time.
    /// Reject stale/future pairs and samples crossing a known reference-generation change.
    pub(crate) fn collector_snapshot(
        &mut self,
        received_reference_ns: i64,
        received_steady_ns: u64,
        received_uncertainty_ns: u64,
    ) -> Result<ObservationSnapshot, ChronosRuntimeError> {
        let mut snapshot = self.snapshot()?;
        let invalid = |reason: &str| ChronosError::InvalidObservation(reason.into());
        let elapsed = snapshot
            .clock
            .steady
            .timestamp_ns
            .checked_sub(received_steady_ns)
            .ok_or_else(|| invalid("collector receive time is in the future"))?;
        if elapsed > 100_000_000 {
            return Err(invalid("collector observation is older than 100ms").into());
        }
        if self
            .last_reference_change_steady
            .is_some_and(|change| received_steady_ns < change.timestamp_ns)
        {
            return Err(invalid("collector observation predates a reference clock change").into());
        }
        let reference = &mut snapshot.clock.reference;
        let difference = (i128::from(reference.timestamp_ns)
            - i128::from(received_reference_ns)
            - i128::from(elapsed))
        .abs();
        let uncertainty = reference.uncertainty_ns.max(received_uncertainty_ns);
        if received_reference_ns < 0
            || received_uncertainty_ns == 0
            || uncertainty > self.config.reference.max_uncertainty_ns
            || difference
                > i128::from(self.config.reference.max_clock_step_ns) + i128::from(uncertainty)
        {
            return Err(
                invalid("collector clock pair is inconsistent with the local reference").into(),
            );
        }
        reference.timestamp_ns = received_reference_ns;
        reference.uncertainty_ns = uncertainty;
        snapshot.clock.steady.timestamp_ns = received_steady_ns;
        Ok(snapshot)
    }

    /// Read and atomically apply one clock pair to global runtime quality.
    fn sample_and_apply(&mut self) -> Result<VirtualClockSnapshot, ChronosRuntimeError> {
        let result = self.try_sample_and_apply();
        if let Err(error) = &result {
            self.record_refresh_failure(error);
        }
        result
    }

    /// Stage one clock pair before committing core and report state.
    fn try_sample_and_apply(&mut self) -> Result<VirtualClockSnapshot, ChronosRuntimeError> {
        let snapshot = self.clock.snapshot()?;
        self.apply_clock_snapshot(&snapshot)?;
        self.latest_quality =
            Some(self.build_quality_report(snapshot.reference.clone(), snapshot.steady)?);
        self.last_successful_refresh_steady = Some(snapshot.steady);
        self.refresh_failure = None;
        Ok(snapshot)
    }

    /// Sample clocks, propagate reference generation, and refresh all quality.
    pub fn refresh(&mut self) -> Result<&ClockQualityReport, ChronosRuntimeError> {
        self.sample_and_apply()?;
        Ok(self
            .latest_quality
            .as_ref()
            .expect("quality was stored before returning"))
    }

    /// Return the last coherent report without performing any clock reads.
    pub fn latest_quality_report(&self) -> Option<&ClockQualityReport> {
        self.latest_quality.as_ref()
    }

    /// Return the latest report together with continuing refresh health.
    pub fn latest_quality_status(&self) -> QualityRuntimeStatus {
        QualityRuntimeStatus {
            report: self.latest_quality.clone(),
            last_successful_refresh_steady: self.last_successful_refresh_steady,
            refresh_failure: self.refresh_failure.clone(),
        }
    }

    /// Build a transport-neutral quality snapshot without reading clocks.
    pub fn quality_snapshot(&self) -> ClockQualitySnapshot {
        ClockQualitySnapshot::from(&self.latest_quality_status())
    }

    /// Build a coherent model snapshot from the latest quality report.
    ///
    /// Current domain identity remains separate from retained coefficients so
    /// a consumer can recognize a model from an older device or reference
    /// generation. Before the first successful clock report this returns
    /// `None` rather than inventing a publication timestamp.
    pub fn models_snapshot(&self) -> Result<Option<ClockModelsSnapshot>, ChronosRuntimeError> {
        let Some(report) = self.latest_quality.as_ref() else {
            return Ok(None);
        };
        let mut domains = Vec::new();
        for domain in &report.domains {
            let DomainQualityReport::AffineModel(status) = domain else {
                continue;
            };
            let model = match self.core.model(&status.domain_id) {
                Ok(mut model) => {
                    if self.refresh_failure.is_some() {
                        degrade_model_for_publication(&mut model);
                    }
                    Some(model)
                }
                Err(ChronosError::ModelUnavailable(_)) => None,
                Err(error) => return Err(error.into()),
            };
            domains.push(ClockModelSnapshot::new(status, model.as_ref()));
        }
        Ok(Some(ClockModelsSnapshot {
            published_steady_time: report.steady,
            reference_generation: report.reference.generation,
            domains,
        }))
    }

    /// Refresh quality without propagating a failure to the driving loop.
    ///
    /// A failed refresh retains the last reference value, marks its current
    /// quality unavailable, and keeps existing conversion coefficients accessible.
    /// The returned status always records the failure even when no prior report
    /// exists yet.
    pub fn evaluate_quality(&mut self) -> QualityRuntimeStatus {
        let _ = self.sample_and_apply();
        self.latest_quality_status()
    }

    /// Pair one configured device timestamp with host receive reference time.
    ///
    /// The observation's fixed receive delay is subtracted from the reference
    /// reading. Device, receive, and reference uncertainties are combined
    /// conservatively. The raw device counter remains unchanged so declared
    /// wrap semantics continue to apply.
    pub fn observe_device_timestamp(
        &mut self,
        domain_id: &str,
        sequence_id: u64,
        device_timestamp_ns: i64,
    ) -> Result<DomainStatus, ChronosRuntimeError> {
        if self.observation_config(domain_id)?.0.query.is_some() {
            return Err(ChronosError::InvalidObservation(
                "hardware query domain requires a query clock pair from its configured adapter"
                    .into(),
            )
            .into());
        }
        let snapshot = self.snapshot()?;
        self.observe_device_timestamp_at(domain_id, sequence_id, device_timestamp_ns, snapshot)
    }

    /// Submit a device timestamp paired with a snapshot taken at message ingress.
    ///
    /// The configured sampling interval is enforced against the snapshot's
    /// steady time. An unavailable reference is propagated to every affine
    /// model and the sample is rejected without discarding retained estimates.
    pub fn observe_device_timestamp_at(
        &mut self,
        domain_id: &str,
        sequence_id: u64,
        device_timestamp_ns: i64,
        snapshot: ObservationSnapshot,
    ) -> Result<DomainStatus, ChronosRuntimeError> {
        let (observation, timestamp_uncertainty_ns) = self.observation_config(domain_id)?;
        if !Arc::ptr_eq(&snapshot.owner, &self.observation_owner) {
            return Err(ChronosRuntimeError::ForeignObservationSnapshot);
        }
        let snapshot_domain_generation = snapshot
            .domain_generations
            .get(domain_id)
            .copied()
            .ok_or_else(|| ChronosRuntimeError::UnknownDomain(domain_id.to_string()))?;
        let snapshot = snapshot.clock;
        let current_generation = self.core.reference_generation();
        if snapshot.reference.generation != current_generation {
            return Err(
                ChronosRuntimeError::ObservationReferenceGenerationMismatch {
                    current: current_generation,
                    snapshot: snapshot.reference.generation,
                },
            );
        }
        let current_domain_generation = self.core.domain_generation(domain_id)?;
        if snapshot_domain_generation != current_domain_generation {
            return Err(ChronosRuntimeError::ObservationDomainGenerationMismatch {
                domain_id: domain_id.to_string(),
                current: current_domain_generation,
                snapshot: snapshot_domain_generation,
            });
        }
        let reference_timestamp = adjusted_reference_timestamp(
            snapshot.reference.timestamp_ns,
            observation.reference_at,
            observation.fixed_delay_ns,
        );
        let reference_timestamp_ns = reference_timestamp?;
        if let Some(previous) = self.last_observation_sample_ns.get(domain_id) {
            let elapsed = snapshot
                .steady
                .timestamp_ns
                .checked_sub(*previous)
                .ok_or(ChronosError::ObservationOutOfOrder)?;
            if elapsed < observation.sampling_interval_ns {
                return Err(ChronosRuntimeError::ObservationIntervalNotElapsed {
                    domain_id: domain_id.to_string(),
                    remaining_ns: observation.sampling_interval_ns - elapsed,
                });
            }
        }
        self.last_observation_sample_ns
            .insert(domain_id.to_string(), snapshot.steady.timestamp_ns);
        if !snapshot.reference.available {
            return Err(ChronosError::ReferenceUnavailable.into());
        }
        let uncertainty_ns = snapshot
            .reference
            .uncertainty_ns
            .saturating_add(timestamp_uncertainty_ns)
            .saturating_add(observation.uncertainty_ns);
        let observation_result = self.core.observe(
            domain_id,
            ClockObservation {
                sequence_id,
                device_timestamp_ns,
                reference_timestamp_ns,
                uncertainty_ns,
                steady_received_ns: snapshot.steady.timestamp_ns,
            },
        );
        let (current_reference, current_steady) = self
            .latest_quality
            .as_ref()
            .map(|report| (report.reference.clone(), report.steady))
            .expect("an observation snapshot stores runtime quality before issue");
        self.core.refresh(current_steady.timestamp_ns)?;
        self.latest_quality = Some(self.build_quality_report(current_reference, current_steady)?);
        Ok(observation_result?)
    }

    /// Start a new affine-domain generation after a confirmed device restart.
    ///
    /// This permits a restarted adapter sequence and device timestamp without
    /// weakening the normal ordering checks. The reset and resulting quality
    /// report use one clock snapshot and preserve reference-generation order.
    pub fn reset_device_domain(
        &mut self,
        domain_id: &str,
    ) -> Result<DomainStatus, ChronosRuntimeError> {
        let domain = self
            .config
            .domains
            .get(domain_id)
            .ok_or_else(|| ChronosRuntimeError::UnknownDomain(domain_id.to_string()))?;
        if matches!(domain.strategy, DomainStrategyConfig::Direct { .. }) {
            return Err(ChronosRuntimeError::DirectDomain(domain_id.to_string()));
        }
        let snapshot = self.sample_and_apply()?;
        let status = self.core.reset(domain_id)?;
        self.last_observation_sample_ns.remove(domain_id);
        self.latest_quality = Some(self.build_quality_report(snapshot.reference, snapshot.steady)?);
        Ok(status)
    }

    /// Return the latest affine model, or `None` before one has been fitted.
    pub fn model(&mut self, domain_id: &str) -> Result<Option<ClockModel>, ChronosRuntimeError> {
        let domain = self
            .config
            .domains
            .get(domain_id)
            .ok_or_else(|| ChronosRuntimeError::UnknownDomain(domain_id.to_string()))?;
        if matches!(domain.strategy, DomainStrategyConfig::Direct { .. }) {
            return Err(ChronosRuntimeError::DirectDomain(domain_id.to_string()));
        }
        self.evaluate_quality();
        match self.core.model(domain_id) {
            Ok(mut model) => {
                if self.refresh_failure.is_some() {
                    if model.state != DomainState::Fault {
                        model.state = DomainState::Unsynced;
                    }
                    model.uncertainty_ns = u64::MAX;
                }
                Ok(Some(model))
            }
            Err(ChronosError::ModelUnavailable(DomainState::Initializing | DomainState::Reset)) => {
                Ok(None)
            }
            Err(error) => Err(error.into()),
        }
    }

    /// Perform a low-frequency best-effort conversion for a modeled domain.
    pub fn convert(
        &mut self,
        domain_id: &str,
        device_timestamp_ns: i64,
    ) -> Result<ConversionResult, ChronosRuntimeError> {
        let domain = self
            .config
            .domains
            .get(domain_id)
            .ok_or_else(|| ChronosRuntimeError::UnknownDomain(domain_id.to_string()))?;
        if matches!(domain.strategy, DomainStrategyConfig::Direct { .. }) {
            return Err(ChronosRuntimeError::DirectDomain(domain_id.to_string()));
        }
        self.evaluate_quality();
        let mut result = self.core.convert(domain_id, device_timestamp_ns)?;
        if self.refresh_failure.is_some() {
            if result.state != DomainState::Fault {
                result.state = DomainState::Unsynced;
            }
            result.uncertainty_ns = u64::MAX;
        }
        Ok(result)
    }

    /// Keep reference generation and model freshness ordered for every sample.
    fn apply_clock_snapshot(
        &mut self,
        snapshot: &VirtualClockSnapshot,
    ) -> Result<(), ChronosRuntimeError> {
        let mut updated = self.core.clone();
        updated.update_reference_generation(snapshot.reference.generation)?;
        updated.update_reference_availability(snapshot.reference.available)?;
        updated.refresh(snapshot.steady.timestamp_ns)?;
        if snapshot.reference.generation != self.core.reference_generation() {
            self.last_reference_change_steady = Some(snapshot.steady);
        }
        self.core = updated;
        Ok(())
    }

    /// Return cloned affine observation settings after strategy validation.
    fn observation_config(
        &self,
        domain_id: &str,
    ) -> Result<(ObservationConfig, u64), ChronosRuntimeError> {
        let domain = self
            .config
            .domains
            .get(domain_id)
            .ok_or_else(|| ChronosRuntimeError::UnknownDomain(domain_id.to_string()))?;
        match &domain.strategy {
            DomainStrategyConfig::Direct { .. } => {
                Err(ChronosRuntimeError::DirectDomain(domain_id.to_string()))
            }
            DomainStrategyConfig::AffineModel { observation, .. } => {
                Ok((observation.clone(), domain.timestamp.uncertainty_ns))
            }
        }
    }

    /// Retain the last report and record a failed periodic refresh.
    ///
    /// Any failed refresh makes current clock quality unknown. Retained
    /// coefficients remain usable, but are downgraded to unbounded quality at
    /// the newest steady time that can still be read.
    fn record_refresh_failure(&mut self, error: &ChronosRuntimeError) {
        let steady = self.clock.steady_time().ok();
        let degradation_error = self.apply_refresh_failure(error, steady);
        let previous = self.refresh_failure.take();
        let message = degradation_error.map_or_else(
            || error.to_string(),
            |degradation_error| {
                format!("{error}; failed to apply conservative quality: {degradation_error}")
            },
        );
        self.refresh_failure = Some(QualityRefreshFailure {
            message,
            consecutive_failures: previous
                .as_ref()
                .map_or(1, |failure| failure.consecutive_failures.saturating_add(1)),
            first_observed_steady: previous
                .as_ref()
                .and_then(|failure| failure.first_observed_steady)
                .or(steady),
            last_observed_steady: steady,
        });
    }

    /// Degrade cached evidence after any failed quality refresh.
    fn apply_refresh_failure(
        &mut self,
        error: &ChronosRuntimeError,
        steady: Option<VirtualSteadyTime>,
    ) -> Option<ChronosError> {
        let previous = self.latest_quality.as_ref()?.clone();
        let mut reference = previous.reference.clone();
        reference.generation_changed = false;
        reference.uncertainty_ns = u64::MAX;
        reference.available = false;
        reference.verified = false;
        reference.reason = format!("quality refresh failed: {error}");
        let steady = steady.unwrap_or(previous.steady);
        let mut updated = self.core.clone();
        if let Err(error) = updated.update_reference_availability(false) {
            self.latest_quality = Some(conservative_failure_report(
                previous,
                reference,
                steady,
                error.to_string(),
            ));
            return Some(error);
        }
        if let Err(error) = updated.refresh(steady.timestamp_ns) {
            self.latest_quality = Some(conservative_failure_report(
                previous,
                reference,
                steady,
                error.to_string(),
            ));
            return Some(error);
        }
        self.core = updated;
        self.latest_quality = self
            .build_quality_report(reference.clone(), steady)
            .ok()
            .or_else(|| {
                Some(conservative_failure_report(
                    previous,
                    reference,
                    steady,
                    error.to_string(),
                ))
            });
        None
    }

    /// Combine configured direct quality and fitted affine-domain status.
    fn build_quality_report(
        &self,
        reference: VirtualReferenceTime,
        steady: VirtualSteadyTime,
    ) -> Result<ClockQualityReport, ChronosRuntimeError> {
        let mut domains = Vec::with_capacity(self.config.domains.len());
        for (domain_id, domain) in &self.config.domains {
            let report = match &domain.strategy {
                DomainStrategyConfig::Direct { .. } => {
                    DomainQualityReport::Direct(DirectDomainQuality {
                        domain_id: domain_id.clone(),
                        time_scale: domain.timestamp.time_scale,
                        reference_generation: reference.generation,
                        reference_available: reference.available,
                        reference_verified: reference.verified,
                        uncertainty_ns: reference
                            .uncertainty_ns
                            .saturating_add(domain.timestamp.uncertainty_ns),
                        reason: format!("configured direct alignment; {}", reference.reason),
                    })
                }
                DomainStrategyConfig::AffineModel { .. } => DomainQualityReport::AffineModel(
                    self.core.status(domain_id, steady.timestamp_ns)?,
                ),
            };
            domains.push(report);
        }
        Ok(ClockQualityReport {
            reference,
            steady,
            domains,
        })
    }
}

/// Overlay unknown quality when publication follows any failed refresh.
fn degrade_model_for_publication(model: &mut ClockModel) {
    if model.state != DomainState::Fault {
        model.state = DomainState::Unsynced;
    }
    model.uncertainty_ns = u64::MAX;
}

/// Degrade cached evidence if rebuilding it from the core is impossible.
fn conservative_failure_report(
    mut report: ClockQualityReport,
    reference: VirtualReferenceTime,
    steady: VirtualSteadyTime,
    reason: String,
) -> ClockQualityReport {
    report.reference = reference;
    report.steady = steady;
    for domain in &mut report.domains {
        match domain {
            DomainQualityReport::Direct(domain) => {
                domain.reference_available = false;
                domain.reference_verified = false;
                domain.uncertainty_ns = u64::MAX;
                domain.reason = format!("quality refresh failed: {reason}");
            }
            DomainQualityReport::AffineModel(domain) => {
                if domain.state != DomainState::Fault {
                    domain.state = DomainState::Unsynced;
                }
                domain.uncertainty_ns = domain.uncertainty_ns.map(|_| u64::MAX);
                domain.reason = format!("quality refresh failed: {reason}");
            }
        }
    }
    report
}

/// Correct a host receive timestamp by its configured non-negative delay.
fn adjusted_reference_timestamp(
    reference_timestamp_ns: i64,
    reference_at: ObservationReferenceKind,
    fixed_delay_ns: u64,
) -> Result<i64, ChronosRuntimeError> {
    match reference_at {
        ObservationReferenceKind::HostReceiveTime | ObservationReferenceKind::HostQueryMidpoint => {
            i128::from(reference_timestamp_ns)
                .checked_sub(i128::from(fixed_delay_ns))
                .and_then(|value| i64::try_from(value).ok())
                .ok_or(ChronosRuntimeError::TimestampAdjustmentOverflow)
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::mpsc::{self, Receiver, SyncSender};
    use std::thread;
    use std::time::{Duration, Instant};

    use super::*;
    use crate::quality_runtime::QualityRuntime;

    #[derive(Debug)]
    struct FakeClock {
        snapshots: VecDeque<Result<VirtualClockSnapshot, ReferenceError>>,
        steady: VirtualSteadyTime,
    }

    #[derive(Debug)]
    struct BlockingClock {
        snapshots: VecDeque<VirtualClockSnapshot>,
        sample_count: Arc<AtomicUsize>,
        worker_entered: Option<SyncSender<()>>,
        release_worker: Receiver<()>,
    }

    #[derive(Debug)]
    struct PanicClock {
        sample_count: usize,
        panic_entered: Option<SyncSender<()>>,
    }

    #[derive(Debug)]
    struct RecoveryClock {
        sample_count: usize,
        recovery_entered: Option<SyncSender<()>>,
        allow_recovery: Receiver<()>,
        recovery_published: Option<SyncSender<()>>,
        release_after_assertion: Receiver<()>,
    }

    struct ReleaseOnDrop(Option<SyncSender<()>>);

    impl ReleaseOnDrop {
        /// Send the release exactly once and disarm drop cleanup.
        fn release(&mut self) {
            if let Some(sender) = self.0.take() {
                let _ = sender.send(());
            }
        }
    }

    impl Drop for ReleaseOnDrop {
        fn drop(&mut self) {
            self.release();
        }
    }

    impl ClockSource for RecoveryClock {
        /// Hold before and after recovery so both statuses are observable.
        fn snapshot(&mut self) -> Result<VirtualClockSnapshot, ReferenceError> {
            self.sample_count += 1;
            match self.sample_count {
                1 => Ok(clock_snapshot(1_000, 1, 1)),
                2 => Err(ReferenceError::Overflow),
                3 => {
                    self.recovery_entered.take().unwrap().send(()).unwrap();
                    self.allow_recovery.recv().unwrap();
                    Ok(clock_snapshot(2_000, 2, 1))
                }
                4 => {
                    self.recovery_published.take().unwrap().send(()).unwrap();
                    self.release_after_assertion.recv().unwrap();
                    Err(ReferenceError::Overflow)
                }
                _ => Err(ReferenceError::Overflow),
            }
        }

        fn steady_time(&self) -> Result<VirtualSteadyTime, ReferenceError> {
            Ok(VirtualSteadyTime { timestamp_ns: 100 })
        }
    }

    impl ClockSource for PanicClock {
        /// Return one initial sample, then simulate an unexpected worker panic.
        fn snapshot(&mut self) -> Result<VirtualClockSnapshot, ReferenceError> {
            self.sample_count += 1;
            if self.sample_count > 1 {
                self.panic_entered.take().unwrap().send(()).unwrap();
                panic!("test quality worker panic");
            }
            Ok(clock_snapshot(1_000, 1, 1))
        }

        fn steady_time(&self) -> Result<VirtualSteadyTime, ReferenceError> {
            Ok(VirtualSteadyTime { timestamp_ns: 100 })
        }
    }

    impl ClockSource for BlockingClock {
        /// Block the second sample so a stop can race with active evaluation.
        fn snapshot(&mut self) -> Result<VirtualClockSnapshot, ReferenceError> {
            let sample_number = self.sample_count.fetch_add(1, Ordering::SeqCst) + 1;
            if sample_number == 2 {
                self.worker_entered.take().unwrap().send(()).unwrap();
                self.release_worker.recv().unwrap();
            }
            self.snapshots.pop_front().ok_or(ReferenceError::Overflow)
        }

        fn steady_time(&self) -> Result<VirtualSteadyTime, ReferenceError> {
            Ok(VirtualSteadyTime { timestamp_ns: 100 })
        }
    }

    impl ClockSource for FakeClock {
        fn snapshot(&mut self) -> Result<VirtualClockSnapshot, ReferenceError> {
            self.snapshots
                .pop_front()
                .unwrap_or(Err(ReferenceError::Overflow))
        }

        fn steady_time(&self) -> Result<VirtualSteadyTime, ReferenceError> {
            Ok(self.steady)
        }
    }

    /// Build one available deterministic clock pair.
    fn clock_snapshot(
        reference_timestamp_ns: i64,
        steady_timestamp_ns: u64,
        generation: u64,
    ) -> VirtualClockSnapshot {
        VirtualClockSnapshot {
            reference: VirtualReferenceTime {
                timestamp_ns: reference_timestamp_ns,
                generation,
                generation_changed: generation > 1,
                uncertainty_ns: 1,
                available: true,
                verified: false,
                reason: "test reference".to_string(),
            },
            steady: VirtualSteadyTime {
                timestamp_ns: steady_timestamp_ns,
            },
        }
    }

    /// Build a runtime whose clock reads are fully deterministic.
    fn runtime_with_snapshots(snapshots: Vec<VirtualClockSnapshot>) -> Chronos {
        runtime_with_results(snapshots.into_iter().map(Ok).collect())
    }

    /// Build a runtime with successful and failed scripted clock reads.
    fn runtime_with_results(
        snapshots: Vec<Result<VirtualClockSnapshot, ReferenceError>>,
    ) -> Chronos {
        let config = ChronosConfig::from_yaml(
            r#"
chronos:
  core:
    reference:
      kind: linux_system
    quality:
      refresh_interval_ns: 1000000
      warning_interval_ns: 1000000000
    domains:
      device_clock:
        capabilities:
          - provider_id: device
            contract_id: robonix/primitive/device/time
            transport: ros2
            timestamp_field: header.stamp
        timestamp:
          event: acquisition
          hardware: true
          time_scale: device_uptime
          resolution_ns: 1
          uncertainty_ns: 1
        strategy:
          kind: affine_model
          observation:
            reference_at: host_receive_time
            sampling_interval_ns: 1
            fixed_delay_ns: 10
            uncertainty_ns: 1
          policy:
            minimum_observations: 2
            observation_window: 4
            max_drift_ppm: 1000000
            max_residual_ns: 1000
            tracking_timeout_ns: 10
            holdover_duration_ns: 10
            max_uncertainty_ns: 1000
"#,
        )
        .unwrap();
        let clock = FakeClock {
            snapshots: snapshots.into(),
            steady: VirtualSteadyTime { timestamp_ns: 100 },
        };
        Chronos::with_clock(config.chronos.core, Box::new(clock)).unwrap()
    }

    #[test]
    /// Fifty milliseconds of helper forwarding cannot move the paired reference or global quality.
    fn collector_pair_excludes_forwarding_delay() {
        let mut chronos = runtime_with_snapshots(vec![
            clock_snapshot(1_000_000_100, 100_000_000, 1),
            clock_snapshot(2_000_000_100, 1_100_000_000, 1),
        ]);
        for (sequence, reference, steady) in [
            (1, 950_000_100, 50_000_000),
            (2, 1_950_000_100, 1_050_000_000),
        ] {
            let snapshot = chronos.collector_snapshot(reference, steady, 100).unwrap();
            assert_eq!(snapshot.reference().timestamp_ns, reference);
            assert_eq!(snapshot.steady().timestamp_ns, steady);
            assert_eq!(snapshot.reference().uncertainty_ns, 100);
            chronos
                .observe_device_timestamp_at(
                    "device_clock",
                    sequence,
                    sequence as i64 * 1_000_000_000,
                    snapshot,
                )
                .unwrap();
        }
        assert_eq!(
            chronos
                .core
                .convert("device_clock", 2_000_000_000)
                .unwrap()
                .reference_timestamp_ns,
            1_950_000_090
        );
        assert_eq!(
            chronos.latest_quality.as_ref().unwrap().steady.timestamp_ns,
            1_100_000_000
        );
        assert_eq!(
            chronos
                .latest_quality
                .as_ref()
                .unwrap()
                .reference
                .timestamp_ns,
            2_000_000_100
        );
    }

    #[test]
    /// Delayed, future, uncertain and inconsistent helper samples are rejected before fitting.
    fn collector_pair_rejects_invalid_clock_evidence() {
        for (reference, steady, uncertainty) in [
            (1_000_000_000, 99_999_999, 1),
            (1_000_000_000, 200_000_001, 1),
            (700_000_000, 150_000_000, 1),
            (950_000_000, 150_000_000, 0),
            (950_000_000, 150_000_000, 1_000_001),
        ] {
            let mut chronos =
                runtime_with_snapshots(vec![clock_snapshot(1_000_000_000, 200_000_000, 1)]);
            assert!(matches!(
                chronos.collector_snapshot(reference, steady, uncertainty),
                Err(ChronosRuntimeError::Core(ChronosError::InvalidObservation(
                    _
                )))
            ));
            assert!(matches!(
                chronos.core.model("device_clock"),
                Err(ChronosError::ModelUnavailable(_))
            ));
        }
    }

    #[test]
    /// A previously detected clock step remains a boundary for helper samples after later refreshes.
    fn collector_pair_cannot_cross_reference_generation() {
        let mut chronos = runtime_with_snapshots(vec![
            clock_snapshot(1_000, 100, 1),
            clock_snapshot(2_000, 200, 2),
            clock_snapshot(2_100, 300, 2),
            clock_snapshot(2_200, 400, 2),
        ]);
        chronos.refresh().unwrap();
        chronos.refresh().unwrap();
        assert!(chronos.collector_snapshot(1_090, 190, 1).is_err());
        let fresh = chronos.collector_snapshot(2_150, 350, 1).unwrap();
        assert_eq!(fresh.reference().generation, 2);
        assert_eq!(fresh.steady().timestamp_ns, 350);
    }

    #[test]
    /// Known receive delay is subtracted from the paired reference timestamp.
    fn host_receive_delay_is_subtracted() {
        assert_eq!(
            adjusted_reference_timestamp(10_000, ObservationReferenceKind::HostReceiveTime, 250,)
                .unwrap(),
            9_750
        );
    }

    #[test]
    /// Delay correction rejects values outside the signed timestamp range.
    fn host_receive_delay_rejects_i64_underflow() {
        assert!(matches!(
            adjusted_reference_timestamp(i64::MIN, ObservationReferenceKind::HostReceiveTime, 1,),
            Err(ChronosRuntimeError::TimestampAdjustmentOverflow)
        ));
    }

    #[test]
    /// Conversion refreshes freshness before returning retained coefficients.
    fn conversion_refreshes_stale_model_quality() {
        let mut chronos = runtime_with_snapshots(vec![
            clock_snapshot(1_000, 1, 1),
            clock_snapshot(2_000, 2, 1),
            clock_snapshot(30_000, 30, 1),
        ]);
        chronos
            .observe_device_timestamp("device_clock", 1, 1_000)
            .unwrap();
        chronos
            .observe_device_timestamp("device_clock", 2, 2_000)
            .unwrap();

        let converted = chronos.convert("device_clock", 2_000).unwrap();
        assert_eq!(converted.state, DomainState::Unsynced);
    }

    #[test]
    /// A reference step keeps retained model identity out of current status.
    fn reference_generation_change_keeps_old_model_identity_separate() {
        let mut chronos = runtime_with_snapshots(vec![
            clock_snapshot(1_000, 1, 1),
            clock_snapshot(2_000, 2, 1),
            clock_snapshot(3_000, 3, 2),
        ]);
        chronos
            .observe_device_timestamp("device_clock", 1, 1_000)
            .unwrap();
        chronos
            .observe_device_timestamp("device_clock", 2, 2_000)
            .unwrap();
        chronos.refresh().unwrap();

        let status = chronos.core.status("device_clock", 3).unwrap();
        assert_eq!(status.state, DomainState::Reset);
        assert_eq!(status.reference_generation, 2);
        assert_eq!(status.uncertainty_ns, None);
        let retained = chronos.core.model("device_clock").unwrap();
        assert_eq!(retained.reference_generation(), 1);
        assert_eq!(retained.state(), DomainState::Unsynced);

        let published = chronos.models_snapshot().unwrap().unwrap();
        let domain = &published.domains[0];
        assert_eq!(domain.state, DomainState::Reset);
        assert_eq!(domain.reference_generation, 2);
        assert_eq!(domain.model.as_ref().unwrap().reference_generation, 1);
    }

    #[test]
    /// A rejected clock pair cannot partially advance core generation.
    fn invalid_snapshot_does_not_partially_change_core_state() {
        let mut chronos = runtime_with_snapshots(vec![
            clock_snapshot(1_000, 1, 1),
            clock_snapshot(2_000, 2, 1),
            clock_snapshot(3_000, 1, 2),
        ]);
        chronos
            .observe_device_timestamp("device_clock", 1, 1_000)
            .unwrap();
        chronos
            .observe_device_timestamp("device_clock", 2, 2_000)
            .unwrap();

        assert!(matches!(
            chronos.refresh(),
            Err(ChronosRuntimeError::Core(
                ChronosError::ObservationOutOfOrder
            ))
        ));
        assert_eq!(chronos.core.reference_generation(), 1);
        let retained = chronos.core.model("device_clock").unwrap();
        assert_eq!(retained.state(), DomainState::Unsynced);
        assert_eq!(retained.uncertainty_ns(), u64::MAX);
        assert!(chronos.latest_quality_status().report_is_historical());
    }

    #[test]
    /// An ingress token uses configured delay and uncertainty exactly once.
    fn ingress_snapshot_builds_exact_model() {
        let mut chronos = runtime_with_snapshots(vec![
            clock_snapshot(1_010, 1, 1),
            clock_snapshot(2_010, 2, 1),
        ]);
        let first = chronos.snapshot().unwrap();
        chronos
            .observe_device_timestamp_at("device_clock", 1, 1_000, first)
            .unwrap();
        let second = chronos.snapshot().unwrap();
        chronos
            .observe_device_timestamp_at("device_clock", 2, 2_000, second)
            .unwrap();

        let model = chronos.core.model("device_clock").unwrap();
        assert_eq!(model.state(), DomainState::Tracking);
        assert_eq!(model.uncertainty_ns(), 4);
        assert_eq!(
            chronos
                .core
                .convert("device_clock", 2_500)
                .unwrap()
                .reference_timestamp_ns,
            2_500
        );
    }

    #[test]
    /// Sampling cadence rejects a token whose exact interval has not elapsed.
    fn sampling_interval_is_enforced() {
        let mut chronos = runtime_with_snapshots(vec![
            clock_snapshot(1_010, 1, 1),
            clock_snapshot(1_020, 1, 1),
        ]);
        let first = chronos.snapshot().unwrap();
        chronos
            .observe_device_timestamp_at("device_clock", 1, 1_000, first)
            .unwrap();
        let second = chronos.snapshot().unwrap();
        assert!(matches!(
            chronos.observe_device_timestamp_at("device_clock", 2, 1_010, second),
            Err(ChronosRuntimeError::ObservationIntervalNotElapsed {
                remaining_ns: 1,
                ..
            })
        ));
    }

    #[test]
    /// A queued old token cannot roll current reference quality backward.
    fn queued_snapshot_does_not_rollback_quality() {
        let mut unavailable = clock_snapshot(2_000, 2, 1);
        unavailable.reference.available = false;
        unavailable.reference.reason = "test reference unavailable".to_string();
        let mut chronos = runtime_with_snapshots(vec![clock_snapshot(1_000, 1, 1), unavailable]);
        let queued = chronos.snapshot().unwrap();
        chronos.refresh().unwrap();
        assert!(matches!(
            chronos.observe_device_timestamp_at("device_clock", 1, 1_000, queued),
            Err(ChronosRuntimeError::Core(
                ChronosError::ReferenceUnavailable
            ))
        ));

        let report = chronos.latest_quality_report().unwrap();
        assert_eq!(report.steady.timestamp_ns, 2);
        assert!(!report.reference.available);
    }

    #[test]
    /// Mature models retain best-effort conversion across reference outage.
    fn unavailable_reference_preserves_and_recovers_model() {
        let mut unavailable = clock_snapshot(3_010, 3, 1);
        unavailable.reference.available = false;
        let unavailable_again = unavailable.clone();
        let mut chronos = runtime_with_snapshots(vec![
            clock_snapshot(1_010, 1, 1),
            clock_snapshot(2_010, 2, 1),
            unavailable,
            unavailable_again,
            clock_snapshot(4_010, 4, 1),
        ]);
        chronos
            .observe_device_timestamp("device_clock", 1, 1_000)
            .unwrap();
        chronos
            .observe_device_timestamp("device_clock", 2, 2_000)
            .unwrap();
        let degraded = chronos.convert("device_clock", 2_000).unwrap();
        assert_eq!(degraded.state, DomainState::Unsynced);
        assert_eq!(degraded.uncertainty_ns, u64::MAX);
        let model = chronos.model("device_clock").unwrap().unwrap();
        assert_eq!(model.state(), DomainState::Unsynced);
        assert_eq!(model.uncertainty_ns(), u64::MAX);

        let recovered = chronos
            .observe_device_timestamp("device_clock", 3, 4_000)
            .unwrap();
        assert_eq!(recovered.state, DomainState::Tracking);
    }

    #[test]
    /// Tokens are accepted only by the Chronos instance that issued them.
    fn foreign_observation_snapshot_is_rejected() {
        let mut first = runtime_with_snapshots(vec![clock_snapshot(1_000, 1, 1)]);
        let token = first.snapshot().unwrap();
        let mut second = runtime_with_snapshots(Vec::new());
        assert!(matches!(
            second.observe_device_timestamp_at("device_clock", 1, 1_000, token),
            Err(ChronosRuntimeError::ForeignObservationSnapshot)
        ));
    }

    #[test]
    /// A queued token from an earlier reference generation is rejected.
    fn old_generation_observation_snapshot_is_rejected() {
        let mut chronos = runtime_with_snapshots(vec![
            clock_snapshot(1_000, 1, 1),
            clock_snapshot(2_000, 2, 2),
        ]);
        let old = chronos.snapshot().unwrap();
        chronos.refresh().unwrap();
        assert!(matches!(
            chronos.observe_device_timestamp_at("device_clock", 1, 1_000, old),
            Err(
                ChronosRuntimeError::ObservationReferenceGenerationMismatch {
                    current: 2,
                    snapshot: 1,
                }
            )
        ));
    }

    #[test]
    /// A confirmed device reset invalidates tokens issued before that reset.
    fn reset_invalidates_queued_observation_snapshot() {
        let mut chronos = runtime_with_snapshots(vec![
            clock_snapshot(1_000, 1, 1),
            clock_snapshot(2_000, 2, 1),
        ]);
        let old = chronos.snapshot().unwrap();
        chronos.reset_device_domain("device_clock").unwrap();
        assert!(matches!(
            chronos.observe_device_timestamp_at("device_clock", 1, 1_000, old),
            Err(ChronosRuntimeError::ObservationDomainGenerationMismatch {
                current: 2,
                snapshot: 1,
                ..
            })
        ));
    }

    #[test]
    /// Delay failure leaves the report created when the token was issued.
    fn delay_overflow_keeps_cached_quality_coherent() {
        let mut chronos = runtime_with_snapshots(vec![clock_snapshot(i64::MIN, 1, 1)]);
        let token = chronos.snapshot().unwrap();
        let expected_reference = token.reference().clone();
        assert!(matches!(
            chronos.observe_device_timestamp_at("device_clock", 1, 1_000, token),
            Err(ChronosRuntimeError::TimestampAdjustmentOverflow)
        ));
        assert_eq!(
            chronos.latest_quality_report().unwrap().reference,
            expected_reference
        );
    }

    #[test]
    /// The composition object can be transferred into a worker thread.
    fn chronos_is_send() {
        fn assert_send<T: Send>() {}
        assert_send::<Chronos>();
    }

    #[test]
    /// Refresh errors retain models, accumulate diagnostics, and recover.
    fn best_effort_quality_refresh_preserves_service() {
        let mut chronos = runtime_with_results(vec![
            Ok(clock_snapshot(1_010, 1, 1)),
            Ok(clock_snapshot(2_010, 2, 1)),
            Err(ReferenceError::Overflow),
            Err(ReferenceError::Overflow),
            Ok(clock_snapshot(3_010, 101, 1)),
        ]);
        chronos
            .observe_device_timestamp("device_clock", 1, 1_000)
            .unwrap();
        chronos
            .observe_device_timestamp("device_clock", 2, 2_000)
            .unwrap();

        let model = chronos.model("device_clock").unwrap().unwrap();
        assert_eq!(model.state(), DomainState::Unsynced);
        assert_eq!(model.uncertainty_ns(), u64::MAX);
        let first_failure = chronos.latest_quality_status();
        assert!(first_failure.report_is_historical());
        let DomainQualityReport::AffineModel(published_status) =
            &first_failure.report.as_ref().unwrap().domains[0]
        else {
            panic!("device_clock must use an affine model");
        };
        assert_eq!(published_status.model_version, model.model_version());
        assert_eq!(
            first_failure
                .refresh_failure
                .as_ref()
                .unwrap()
                .consecutive_failures,
            1
        );
        assert!(!first_failure.report.as_ref().unwrap().reference.available);
        let published_model = chronos.models_snapshot().unwrap().unwrap();
        assert_eq!(
            published_model.domains[0].model_version,
            published_model.domains[0]
                .model
                .as_ref()
                .unwrap()
                .model_version
        );

        let converted = chronos.convert("device_clock", 2_000).unwrap();
        assert_eq!(converted.state, DomainState::Unsynced);
        assert_eq!(converted.uncertainty_ns, u64::MAX);
        assert_eq!(
            chronos
                .latest_quality_status()
                .refresh_failure
                .unwrap()
                .consecutive_failures,
            2
        );

        let recovered = chronos
            .observe_device_timestamp("device_clock", 3, 3_000)
            .unwrap();
        assert_eq!(recovered.state, DomainState::Tracking);
        assert!(chronos.latest_quality_status().refresh_failure.is_none());
    }

    #[test]
    /// A failed conservative refresh cannot split report and model identity.
    fn fallback_report_keeps_model_identity_coherent() {
        let mut chronos = runtime_with_snapshots(vec![
            clock_snapshot(1_010, 1, 1),
            clock_snapshot(2_010, 2, 1),
        ]);
        chronos
            .observe_device_timestamp("device_clock", 1, 1_000)
            .unwrap();
        chronos
            .observe_device_timestamp("device_clock", 2, 2_000)
            .unwrap();
        chronos.clock = Box::new(FakeClock {
            snapshots: vec![Err(ReferenceError::Overflow)].into(),
            steady: VirtualSteadyTime { timestamp_ns: 1 },
        });

        let status = chronos.evaluate_quality();
        assert!(
            status
                .refresh_failure
                .as_ref()
                .unwrap()
                .message
                .contains("failed to apply conservative quality")
        );
        let published = chronos.models_snapshot().unwrap().unwrap();
        let domain = &published.domains[0];
        let model = domain.model.as_ref().unwrap();
        assert_eq!(domain.generation, model.generation);
        assert_eq!(domain.reference_generation, model.reference_generation);
        assert_eq!(domain.model_version, model.model_version);
        assert_eq!(model.state, DomainState::Unsynced);
        assert_eq!(model.uncertainty_ns, u64::MAX);
    }

    #[test]
    /// An initial clock failure is reportable before any reference value exists.
    fn initial_quality_failure_has_no_fabricated_report() {
        let mut chronos = runtime_with_results(vec![Err(ReferenceError::Overflow)]);
        let status = chronos.evaluate_quality();
        assert!(status.report.is_none());
        assert!(!status.report_is_historical());
        assert_eq!(status.last_successful_refresh_steady, None);
        assert_eq!(status.refresh_failure.unwrap().consecutive_failures, 1);
        let snapshot = chronos.quality_snapshot();
        assert!(snapshot.report.is_none());
        assert!(snapshot.refresh_health.failure.is_some());
        assert!(chronos.models_snapshot().unwrap().is_none());
    }

    #[test]
    /// A refresh failure does not imply an estimate before a model exists.
    fn quality_failure_preserves_missing_model_uncertainty() {
        let mut chronos = runtime_with_results(vec![
            Ok(clock_snapshot(1_000, 1, 1)),
            Err(ReferenceError::Overflow),
            Err(ReferenceError::Overflow),
        ]);
        chronos.refresh().unwrap();
        let status = chronos.evaluate_quality();
        let domain = status
            .report
            .unwrap()
            .domains
            .into_iter()
            .find(|domain| domain.domain_id() == "device_clock")
            .unwrap();
        let DomainQualityReport::AffineModel(domain) = domain else {
            panic!("device_clock must use an affine model");
        };
        assert_eq!(domain.state, DomainState::Unsynced);
        assert_eq!(domain.uncertainty_ns, None);
        assert!(chronos.model("device_clock").unwrap().is_none());
    }

    #[test]
    /// Periodic evaluation clears historical diagnostics after recovery.
    fn quality_runtime_recovers_after_refresh_error() {
        let (recovery_entered_tx, recovery_entered_rx) = mpsc::sync_channel(1);
        let (allow_recovery_tx, allow_recovery_rx) = mpsc::sync_channel(1);
        let (recovery_published_tx, recovery_published_rx) = mpsc::sync_channel(1);
        let (release_tx, release_rx) = mpsc::sync_channel(1);
        let mut chronos = runtime_with_snapshots(Vec::new());
        chronos.clock = Box::new(RecoveryClock {
            sample_count: 0,
            recovery_entered: Some(recovery_entered_tx),
            allow_recovery: allow_recovery_rx,
            recovery_published: Some(recovery_published_tx),
            release_after_assertion: release_rx,
        });
        let runtime = QualityRuntime::start(chronos).unwrap();
        let mut allow_recovery = ReleaseOnDrop(Some(allow_recovery_tx));
        let mut release_after_assertion = ReleaseOnDrop(Some(release_tx));
        recovery_entered_rx
            .recv_timeout(Duration::from_secs(1))
            .unwrap();
        let failed = runtime.latest_quality_status().unwrap();

        allow_recovery.release();
        recovery_published_rx
            .recv_timeout(Duration::from_secs(1))
            .unwrap();
        let recovered = runtime.latest_quality_status().unwrap();
        let stopper = thread::spawn(move || runtime.stop());
        release_after_assertion.release();
        stopper.join().unwrap().unwrap();

        assert!(failed.refresh_failure.is_some());
        assert!(!recovered.report_is_historical());
        assert!(recovered.refresh_failure.is_none());
        assert_eq!(
            recovered.last_successful_refresh_steady,
            Some(VirtualSteadyTime { timestamp_ns: 2 })
        );
        assert_eq!(recovered.report.unwrap().steady.timestamp_ns, 2);
    }

    #[test]
    /// Refresh diagnostics retain exact success and failure steady times.
    fn quality_refresh_records_exact_diagnostic_times() {
        let mut chronos = runtime_with_results(vec![
            Ok(clock_snapshot(1_000, 1, 1)),
            Err(ReferenceError::Overflow),
            Err(ReferenceError::Overflow),
        ]);
        assert!(!chronos.evaluate_quality().report_is_historical());
        let first = chronos.evaluate_quality();
        assert_eq!(
            first.last_successful_refresh_steady,
            Some(VirtualSteadyTime { timestamp_ns: 1 })
        );
        assert_eq!(
            first.refresh_failure.unwrap().first_observed_steady,
            Some(VirtualSteadyTime { timestamp_ns: 100 })
        );
        let second = chronos.evaluate_quality();
        let failure = second.refresh_failure.unwrap();
        assert_eq!(failure.consecutive_failures, 2);
        assert_eq!(
            failure.first_observed_steady,
            Some(VirtualSteadyTime { timestamp_ns: 100 })
        );
        assert_eq!(
            failure.last_observed_steady,
            Some(VirtualSteadyTime { timestamp_ns: 100 })
        );
    }

    #[test]
    /// The facade delegates observation, conversion, reset, and status updates.
    fn quality_runtime_delegates_complete_in_process_api() {
        let mut chronos = runtime_with_snapshots(
            (1..=10)
                .map(|steady| clock_snapshot(i64::from(steady) * 1_000 + 10, steady as u64, 1))
                .collect(),
        );
        chronos.config.quality.refresh_interval_ns = 60_000_000_000;
        let runtime = QualityRuntime::start(chronos).unwrap();
        assert_eq!(
            runtime.config().unwrap().quality.refresh_interval_ns,
            60_000_000_000
        );
        assert_eq!(runtime.steady_time().unwrap().timestamp_ns, 100);
        assert_eq!(runtime.reference_time().unwrap().timestamp_ns, 2_010);
        assert_eq!(
            runtime
                .evaluate_now()
                .unwrap()
                .report
                .unwrap()
                .steady
                .timestamp_ns,
            3
        );
        let first = runtime.snapshot().unwrap();
        runtime
            .observe_device_timestamp_at("device_clock", 1, 4_000, first)
            .unwrap();
        runtime
            .observe_device_timestamp("device_clock", 2, 5_000)
            .unwrap();
        assert!(runtime.model("device_clock").unwrap().is_some());
        assert_eq!(
            runtime.convert("device_clock", 5_000).unwrap().state,
            DomainState::Tracking
        );
        assert!(runtime.quality_snapshot().unwrap().report.is_some());
        let published_models = runtime.models_snapshot().unwrap().unwrap();
        assert_eq!(published_models.domains.len(), 1);
        assert!(published_models.domains[0].model.is_some());
        assert_eq!(
            runtime.reset_device_domain("device_clock").unwrap().state,
            DomainState::Reset
        );
        assert_eq!(
            runtime
                .latest_quality_status()
                .unwrap()
                .report
                .unwrap()
                .steady
                .timestamp_ns,
            8
        );
        runtime.stop().unwrap();
    }

    #[test]
    /// Continuous runtime handles can be shared between consumer threads.
    fn quality_runtime_is_send_and_sync() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<QualityRuntime>();
    }

    #[test]
    /// Stop wakes promptly when requested during an active evaluation.
    fn quality_runtime_stop_does_not_lose_wakeup() {
        let (entered_tx, entered_rx) = mpsc::sync_channel(1);
        let (release_tx, release_rx) = mpsc::sync_channel(1);
        let sample_count = Arc::new(AtomicUsize::new(0));
        let mut chronos = runtime_with_snapshots(Vec::new());
        chronos.config.quality.refresh_interval_ns = 2_000_000_000;
        chronos.clock = Box::new(BlockingClock {
            snapshots: vec![clock_snapshot(1_000, 1, 1), clock_snapshot(2_000, 2, 1)].into(),
            sample_count: sample_count.clone(),
            worker_entered: Some(entered_tx),
            release_worker: release_rx,
        });
        let runtime = QualityRuntime::start(chronos).unwrap();
        let mut release_worker = ReleaseOnDrop(Some(release_tx));
        let stop_signal = runtime.stop_signal_for_test();
        entered_rx.recv_timeout(Duration::from_secs(5)).unwrap();
        let started = Instant::now();
        let stopper = thread::spawn(move || runtime.stop());
        let (stop_lock, stop_wake) = &*stop_signal;
        let stopped = stop_lock.lock().unwrap();
        let (stopped, timeout) = stop_wake
            .wait_timeout_while(stopped, Duration::from_secs(1), |stopped| !*stopped)
            .unwrap();
        let stop_was_observed = *stopped && !timeout.timed_out();
        drop(stopped);
        release_worker.release();
        stopper.join().unwrap().unwrap();
        assert!(stop_was_observed);
        assert!(started.elapsed() < Duration::from_secs(1));
        assert_eq!(sample_count.load(Ordering::SeqCst), 2);
        thread::sleep(Duration::from_millis(250));
        assert_eq!(sample_count.load(Ordering::SeqCst), 2);
    }

    #[test]
    /// Drop uses the same prompt wakeup path as explicit stop.
    fn quality_runtime_drop_does_not_lose_wakeup() {
        let (entered_tx, entered_rx) = mpsc::sync_channel(1);
        let (release_tx, release_rx) = mpsc::sync_channel(1);
        let sample_count = Arc::new(AtomicUsize::new(0));
        let mut chronos = runtime_with_snapshots(Vec::new());
        chronos.config.quality.refresh_interval_ns = 2_000_000_000;
        chronos.clock = Box::new(BlockingClock {
            snapshots: vec![clock_snapshot(1_000, 1, 1), clock_snapshot(2_000, 2, 1)].into(),
            sample_count: sample_count.clone(),
            worker_entered: Some(entered_tx),
            release_worker: release_rx,
        });
        let runtime = QualityRuntime::start(chronos).unwrap();
        let mut release_worker = ReleaseOnDrop(Some(release_tx));
        let stop_signal = runtime.stop_signal_for_test();
        entered_rx.recv_timeout(Duration::from_secs(5)).unwrap();
        let started = Instant::now();
        let dropper = thread::spawn(move || drop(runtime));
        let (stop_lock, stop_wake) = &*stop_signal;
        let stopped = stop_lock.lock().unwrap();
        let (stopped, timeout) = stop_wake
            .wait_timeout_while(stopped, Duration::from_secs(1), |stopped| !*stopped)
            .unwrap();
        let stop_was_observed = *stopped && !timeout.timed_out();
        drop(stopped);
        release_worker.release();
        dropper.join().unwrap();
        assert!(stop_was_observed);
        assert!(started.elapsed() < Duration::from_secs(1));
        assert_eq!(sample_count.load(Ordering::SeqCst), 2);
    }

    #[test]
    /// A panicked worker becomes visible in status and poisons delegated access.
    fn quality_runtime_reports_worker_panic() {
        let (panic_entered_tx, panic_entered_rx) = mpsc::sync_channel(1);
        let mut chronos = runtime_with_snapshots(Vec::new());
        chronos.clock = Box::new(PanicClock {
            sample_count: 0,
            panic_entered: Some(panic_entered_tx),
        });
        let runtime = QualityRuntime::start(chronos).unwrap();
        panic_entered_rx
            .recv_timeout(Duration::from_secs(1))
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(1);
        while !matches!(
            runtime.latest_quality_status(),
            Err(ChronosRuntimeError::QualityStatePoisoned)
        ) && Instant::now() < deadline
        {
            thread::sleep(Duration::from_millis(1));
        }

        assert!(matches!(
            runtime.latest_quality_status(),
            Err(ChronosRuntimeError::QualityStatePoisoned)
        ));
        assert!(matches!(
            runtime.evaluate_now(),
            Err(ChronosRuntimeError::QualityStatePoisoned)
        ));
        assert!(matches!(
            runtime.stop(),
            Err(ChronosRuntimeError::QualityWorkerPanicked)
        ));
    }
}
