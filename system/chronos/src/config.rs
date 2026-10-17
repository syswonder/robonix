// SPDX-License-Identifier: MulanPSL-2.0

use std::collections::{BTreeMap, BTreeSet};

pub use robonix_chronos_clock::{ChronosConfigError, ReferenceConfig, ReferenceSourceKind};
use serde::Deserialize;

use crate::model::{ClockDomainDescriptor, ClockDomainPolicy, validate_domain_id};

/// Transports used to discover a query's driver or a ROS timestamp stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CapabilityTransportKind {
    Ros2,
    Grpc,
}

/// Time scales that a timestamp-producing capability may declare.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TimestampScaleKind {
    LinuxSystem = 0,
    RosTime = 1,
    DeviceUptime = 2,
}

/// Acquisition event represented by a configured timestamp field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TimestampEventKind {
    Acquisition,
}

/// Supported ways to pair a device timestamp with the selected reference.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ObservationReferenceKind {
    HostReceiveTime,
    HostQueryMidpoint,
}

/// Configured Python factory that adapts an existing hardware clock query.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QueryAdapterConfig {
    pub adapter: String,
    pub tick_ns: u64,
    #[serde(default = "default_query_timeout_ns")]
    pub timeout_ns: u64,
    #[serde(default)]
    pub options: BTreeMap<String, String>,
}

fn default_query_timeout_ns() -> u64 {
    50_000_000
}

/// One timestamp stream or existing driver capability used for query readiness.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CapabilityBindingConfig {
    pub provider_id: String,
    pub contract_id: String,
    pub transport: CapabilityTransportKind,
    #[serde(default)]
    pub timestamp_field: String,
    #[serde(default)]
    pub session_id_field: Option<String>,
    #[serde(default)]
    pub device_generation_field: Option<String>,
    #[serde(default)]
    pub sequence_id_field: Option<String>,
}

/// Static meaning and device-level quality of one timestamp field.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TimestampConfig {
    pub event: TimestampEventKind,
    pub hardware: bool,
    pub time_scale: TimestampScaleKind,
    pub resolution_ns: u64,
    #[serde(default)]
    pub wrap_period_ns: Option<u64>,
    pub uncertainty_ns: u64,
}

/// How a local adapter pairs a device counter with the host reference.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObservationConfig {
    pub reference_at: ObservationReferenceKind,
    pub sampling_interval_ns: u64,
    #[serde(default)]
    pub fixed_delay_ns: u64,
    pub uncertainty_ns: u64,
    #[serde(default)]
    pub query: Option<QueryAdapterConfig>,
}

/// Serialized form of the core's affine-model maturity and quality limits.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ModelPolicyConfig {
    pub minimum_observations: usize,
    pub observation_window: usize,
    pub max_drift_ppm: u64,
    pub max_residual_ns: u64,
    pub tracking_timeout_ns: u64,
    pub holdover_duration_ns: u64,
    pub max_uncertainty_ns: u64,
    #[serde(default)]
    pub model_update: crate::model::ModelUpdatePolicy,
}

impl From<&ModelPolicyConfig> for ClockDomainPolicy {
    fn from(value: &ModelPolicyConfig) -> Self {
        Self {
            minimum_observations: value.minimum_observations,
            observation_window: value.observation_window,
            max_drift_ppm: value.max_drift_ppm,
            max_residual_ns: value.max_residual_ns,
            tracking_timeout_ns: value.tracking_timeout_ns,
            holdover_duration_ns: value.holdover_duration_ns,
            max_uncertainty_ns: value.max_uncertainty_ns,
            model_update: value.model_update,
        }
    }
}

/// Explicit choice between trusting a configured alignment and fitting it.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum DomainStrategyConfig {
    Direct {
        aligned_to: ReferenceSourceKind,
    },
    AffineModel {
        observation: ObservationConfig,
        policy: ModelPolicyConfig,
    },
}

/// Complete deployment declaration for one timestamp domain.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DomainConfig {
    #[serde(default)]
    pub capabilities: Vec<CapabilityBindingConfig>,
    pub timestamp: TimestampConfig,
    pub strategy: DomainStrategyConfig,
}

/// Cadence for continuous quality refresh and rate-limited warnings.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QualityConfig {
    #[serde(default = "default_quality_refresh_interval_ns")]
    pub refresh_interval_ns: u64,
    #[serde(default = "default_warning_interval_ns")]
    pub warning_interval_ns: u64,
}

impl Default for QualityConfig {
    fn default() -> Self {
        Self {
            refresh_interval_ns: default_quality_refresh_interval_ns(),
            warning_interval_ns: default_warning_interval_ns(),
        }
    }
}

impl QualityConfig {
    /// Reject zero periods that would create a busy loop or warning flood.
    pub fn validate(&self) -> Result<(), ChronosConfigError> {
        if self.refresh_interval_ns == 0 || self.warning_interval_ns == 0 {
            return Err(ChronosConfigError::InvalidQuality(
                "quality refresh and warning intervals must be positive".to_string(),
            ));
        }
        Ok(())
    }
}

/// Configuration of the privilege-free Chronos core.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CoreConfig {
    pub reference: ReferenceConfig,
    #[serde(default)]
    pub quality: QualityConfig,
    #[serde(default)]
    pub domains: BTreeMap<String, DomainConfig>,
}

impl CoreConfig {
    /// Validate every domain and reject capability bindings assigned twice.
    pub fn validate(&self) -> Result<(), ChronosConfigError> {
        self.reference.validate()?;
        self.quality.validate()?;
        let mut bindings = BTreeSet::new();
        for (domain_id, domain) in &self.domains {
            validate_domain(domain_id, domain, self.reference.kind)?;
            for binding in &domain.capabilities {
                let identity = (
                    binding.provider_id.clone(),
                    binding.contract_id.clone(),
                    binding.transport,
                );
                if !bindings.insert(identity) {
                    return Err(ChronosConfigError::InvalidDomain {
                        domain_id: domain_id.clone(),
                        reason: format!(
                            "capability '{}:{}:{:?}' is assigned more than once",
                            binding.provider_id, binding.contract_id, binding.transport
                        ),
                    });
                }
            }
        }
        Ok(())
    }

    /// Build the pure-core registrations required only by affine-model domains.
    pub fn affine_model_domains(
        &self,
    ) -> Result<Vec<(ClockDomainDescriptor, ClockDomainPolicy)>, ChronosConfigError> {
        self.validate()?;
        Ok(self
            .domains
            .iter()
            .filter_map(|(domain_id, domain)| match &domain.strategy {
                DomainStrategyConfig::Direct { .. } => None,
                DomainStrategyConfig::AffineModel { policy, .. } => Some((
                    ClockDomainDescriptor {
                        domain_id: domain_id.clone(),
                        resolution_ns: domain.timestamp.resolution_ns,
                        wrap_period_ns: domain.timestamp.wrap_period_ns,
                    },
                    ClockDomainPolicy::from(policy),
                )),
            })
            .collect())
    }
}

/// Configuration nested below the top-level `chronos` key.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChronosSection {
    #[serde(default)]
    pub reference_reader_command: Vec<String>,
    #[serde(default)]
    pub collect_observations: bool,
    #[serde(default)]
    pub collect_ros_observations: bool,
    pub core: CoreConfig,
}

impl ChronosSection {
    /// Select automatic query input first; ROS receipt requires both explicit switches.
    pub(crate) fn collects_domain(&self, domain: &DomainConfig) -> bool {
        self.collect_observations
            && matches!(&domain.strategy, DomainStrategyConfig::AffineModel { observation, .. }
                if observation.query.is_some() || self.collect_ros_observations)
    }
}

/// Top-level configuration accepted by the current Chronos library.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChronosConfig {
    pub chronos: ChronosSection,
}

impl ChronosConfig {
    /// Parse and validate one YAML Chronos configuration document.
    pub fn from_yaml(yaml: &str) -> Result<Self, ChronosConfigError> {
        let config: Self = serde_yaml::from_str(yaml)?;
        config.chronos.core.validate()?;
        Ok(config)
    }
}

/// Enforce strategy-specific semantics before any runtime observation starts.
fn validate_domain(
    domain_id: &str,
    domain: &DomainConfig,
    reference: ReferenceSourceKind,
) -> Result<(), ChronosConfigError> {
    let invalid = |reason: &str| ChronosConfigError::InvalidDomain {
        domain_id: domain_id.to_string(),
        reason: reason.to_string(),
    };
    validate_domain_id(domain_id).map_err(invalid)?;
    let query = matches!(&domain.strategy, DomainStrategyConfig::AffineModel { observation, .. }
        if observation.query.is_some());
    if domain.capabilities.is_empty() && !query {
        return Err(invalid("at least one capability is required"));
    }
    for binding in &domain.capabilities {
        let identity_fields = [
            binding.session_id_field.as_ref(),
            binding.device_generation_field.as_ref(),
            binding.sequence_id_field.as_ref(),
        ];
        let count = identity_fields
            .iter()
            .filter(|field| field.is_some())
            .count();
        if count != 0 && count != 3 {
            return Err(invalid(
                "configure all three capture identity field paths, or omit all three",
            ));
        }
        if binding.provider_id.trim().is_empty()
            || binding.contract_id.trim().is_empty()
            || (!query && binding.timestamp_field.trim().is_empty())
        {
            return Err(invalid(
                "provider_id, contract_id, and timestamp_field must not be empty",
            ));
        }
        if !query && binding.transport != CapabilityTransportKind::Ros2 {
            return Err(invalid(
                "timestamp field extraction requires ros2 transport",
            ));
        }
        for path in std::iter::once(
            (!query || !binding.timestamp_field.is_empty()).then_some(&binding.timestamp_field),
        )
        .chain([
            binding.session_id_field.as_ref(),
            binding.device_generation_field.as_ref(),
            binding.sequence_id_field.as_ref(),
        ])
        .flatten()
        {
            if path.split('.').any(|part| {
                part.is_empty()
                    || !part.chars().enumerate().all(|(index, character)| {
                        character == '_'
                            || character.is_ascii_alphabetic()
                            || (index > 0 && character.is_ascii_digit())
                    })
            }) {
                return Err(invalid(
                    "message field paths must contain dotted identifiers",
                ));
            }
        }
    }
    if domain.timestamp.resolution_ns == 0 {
        return Err(invalid("timestamp resolution_ns must be positive"));
    }
    ClockDomainDescriptor {
        domain_id: domain_id.to_string(),
        resolution_ns: domain.timestamp.resolution_ns,
        wrap_period_ns: domain.timestamp.wrap_period_ns,
    }
    .validate()
    .map_err(|error| invalid(&error.to_string()))?;
    match &domain.strategy {
        DomainStrategyConfig::Direct { aligned_to } => {
            if domain.timestamp.time_scale == TimestampScaleKind::DeviceUptime {
                return Err(invalid("device_uptime requires affine_model"));
            }
            if *aligned_to != reference {
                return Err(invalid(
                    "direct alignment must match the selected reference",
                ));
            }
        }
        DomainStrategyConfig::AffineModel {
            observation,
            policy,
        } => {
            if reference == ReferenceSourceKind::RosTime {
                return Err(invalid(
                    "affine_model requires linux_system reference; simulated clock fitting is not supported",
                ));
            }
            if domain.timestamp.time_scale != TimestampScaleKind::DeviceUptime {
                return Err(invalid("affine_model currently requires device_uptime"));
            }
            if observation.sampling_interval_ns == 0 || observation.uncertainty_ns == 0 {
                return Err(invalid(
                    "observation sampling interval and uncertainty must be positive",
                ));
            }
            match (&observation.query, observation.reference_at) {
                (None, ObservationReferenceKind::HostReceiveTime) => {}
                (Some(query), ObservationReferenceKind::HostQueryMidpoint) => {
                    let valid_identifier = |part: &str| {
                        !part.is_empty()
                            && part.chars().enumerate().all(|(index, c)| {
                                c == '_'
                                    || c.is_ascii_alphabetic()
                                    || (index > 0 && c.is_ascii_digit())
                            })
                    };
                    let valid_factory =
                        query
                            .adapter
                            .split_once(':')
                            .is_some_and(|(module, factory)| {
                                module.split('.').all(valid_identifier) && valid_identifier(factory)
                            });
                    if !valid_factory
                        || !(1..=i64::MAX as u64).contains(&query.tick_ns)
                        || !(1..=50_000_000).contains(&query.timeout_ns)
                    {
                        return Err(invalid(
                            "query requires module:factory, positive tick_ns and timeout_ns in 1..=50000000",
                        ));
                    }
                    if observation.fixed_delay_ns != 0 {
                        return Err(invalid(
                            "query midpoint cannot use host receive fixed_delay_ns",
                        ));
                    }
                    if let Some(period) = domain.timestamp.wrap_period_ns {
                        if period % query.tick_ns != 0 || period / query.tick_ns < 2 {
                            return Err(invalid(
                                "query wrap_period_ns must contain at least two whole ticks",
                            ));
                        }
                        let ambiguous_after = (u128::from(period) * 1_000_000)
                            .div_ceil((1_000_000 + u128::from(policy.max_drift_ppm)) * 2);
                        if u128::from(observation.sampling_interval_ns)
                            + u128::from(query.timeout_ns)
                            >= ambiguous_after
                        {
                            return Err(invalid(
                                "query interval plus timeout must stay below half the wrap period including drift",
                            ));
                        }
                    }
                }
                _ => {
                    return Err(invalid(
                        "host_query_midpoint requires query; host_receive_time requires ROS sampling",
                    ));
                }
            }
            ClockDomainPolicy::from(policy)
                .validate()
                .map_err(|error| invalid(&error.to_string()))?;
        }
    }
    Ok(())
}

fn default_quality_refresh_interval_ns() -> u64 {
    1_000_000_000
}

fn default_warning_interval_ns() -> u64 {
    10_000_000_000
}

#[cfg(test)]
mod tests {
    use super::{
        CapabilityTransportKind, ChronosConfig, DomainStrategyConfig, ObservationReferenceKind,
        ReferenceSourceKind, TimestampScaleKind,
    };

    const DOMAIN_CONFIG: &str = r#"
chronos:
  core:
    reference:
      kind: linux_system
    domains:
      front_camera_clock:
        capabilities:
          - provider_id: front_camera
            contract_id: robonix/primitive/camera/rgb
            transport: ros2
            timestamp_field: header.stamp
        timestamp:
          event: acquisition
          hardware: true
          time_scale: ros_time
          resolution_ns: 1
          uncertainty_ns: 1000000
        strategy:
          kind: direct
          aligned_to: linux_system
      lidar_uptime_clock:
        capabilities:
          - provider_id: front_lidar
            contract_id: robonix/primitive/lidar/lidar
            transport: ros2
            timestamp_field: header.stamp
        timestamp:
          event: acquisition
          hardware: true
          time_scale: device_uptime
          resolution_ns: 1000
          wrap_period_ns: null
          uncertainty_ns: 1000
        strategy:
          kind: affine_model
          observation:
            reference_at: host_receive_time
            sampling_interval_ns: 500000000
            fixed_delay_ns: 0
            uncertainty_ns: 5000000
          policy:
            minimum_observations: 8
            observation_window: 32
            max_drift_ppm: 200
            max_residual_ns: 10000000
            tracking_timeout_ns: 2000000000
            holdover_duration_ns: 5000000000
            max_uncertainty_ns: 20000000
"#;

    #[test]
    /// The documented minimal configuration selects Linux system time.
    fn parses_linux_system_reference() {
        let config = ChronosConfig::from_yaml(
            r#"
chronos:
  core:
    reference:
      kind: linux_system
"#,
        )
        .unwrap();
        assert_eq!(
            config.chronos.core.reference.kind,
            ReferenceSourceKind::LinuxSystem
        );
        assert_eq!(config.chronos.core.reference.max_uncertainty_ns, 1_000_000);
        assert!(!config.chronos.collect_observations);
        assert!(!config.chronos.collect_ros_observations);
        assert_eq!(
            config.chronos.core.quality.refresh_interval_ns,
            1_000_000_000
        );
        assert_eq!(
            config.chronos.core.quality.warning_interval_ns,
            10_000_000_000
        );
    }

    #[test]
    /// Unknown and not-yet-implemented source values fail during parsing.
    fn rejects_unknown_reference_source() {
        let error = ChronosConfig::from_yaml(
            r#"
chronos:
  core:
    reference:
      kind: linux_ptp
"#,
        )
        .unwrap_err();
        assert!(error.to_string().contains("linux_ptp"));
    }

    #[test]
    /// A zero freshness or quality limit is never silently accepted.
    fn rejects_zero_reference_limit() {
        let error = ChronosConfig::from_yaml(
            r#"
chronos:
  core:
    reference:
      kind: linux_system
      max_status_age_ns: 0
"#,
        )
        .unwrap_err();
        assert!(error.to_string().contains("must be positive"));
    }

    #[test]
    /// Continuous quality evaluation rejects a zero refresh cadence.
    fn rejects_zero_quality_interval() {
        let error = ChronosConfig::from_yaml(
            r#"
chronos:
  core:
    reference:
      kind: linux_system
    quality:
      refresh_interval_ns: 0
      warning_interval_ns: 1000000000
"#,
        )
        .unwrap_err();
        assert!(error.to_string().contains("quality refresh"));
    }

    #[test]
    /// Direct and modeled domains retain their distinct timestamp semantics.
    fn parses_explicit_domain_strategies() {
        let config = ChronosConfig::from_yaml(DOMAIN_CONFIG).unwrap();
        let direct = &config.chronos.core.domains["front_camera_clock"];
        assert_eq!(direct.timestamp.time_scale, TimestampScaleKind::RosTime);
        assert!(matches!(
            direct.strategy,
            DomainStrategyConfig::Direct { .. }
        ));

        let registrations = config.chronos.core.affine_model_domains().unwrap();
        assert_eq!(registrations.len(), 1);
        assert_eq!(registrations[0].0.domain_id, "lidar_uptime_clock");
        assert_eq!(registrations[0].0.resolution_ns, 1_000);
        assert_eq!(registrations[0].1.minimum_observations, 8);
    }

    #[test]
    /// Unsupported domain IDs fail at configuration loading, before collection starts.
    fn rejects_domain_ids_that_cannot_be_observed() {
        for original in ["front_camera_clock", "lidar_uptime_clock"] {
            for id in [
                String::new(),
                " camera ".into(),
                "x".repeat(129),
                "\u{754c}".repeat(43),
            ] {
                let yaml = DOMAIN_CONFIG.replace(&format!("{original}:"), &format!("'{id}':"));
                let error = ChronosConfig::from_yaml(&yaml).unwrap_err();
                assert!(error.to_string().contains("1..128 bytes"));
            }
            let yaml =
                DOMAIN_CONFIG.replace(&format!("{original}:"), &format!("'{}':", "x".repeat(128)));
            ChronosConfig::from_yaml(&yaml).unwrap();
        }
    }

    #[test]
    /// A device uptime declaration cannot bypass the required affine model.
    fn rejects_direct_device_uptime() {
        let yaml = r#"
chronos:
  core:
    reference:
      kind: linux_system
    domains:
      camera_clock:
        capabilities:
          - provider_id: camera
            contract_id: robonix/primitive/camera/rgb
            transport: ros2
            timestamp_field: header.stamp
        timestamp:
          event: acquisition
          hardware: true
          time_scale: device_uptime
          resolution_ns: 1
          uncertainty_ns: 1
        strategy:
          kind: direct
          aligned_to: linux_system
"#;
        let error = ChronosConfig::from_yaml(yaml).unwrap_err();
        assert!(
            error
                .to_string()
                .contains("device_uptime requires affine_model")
        );
    }

    #[test]
    /// Affine fitting is not accepted for timestamps declared on another scale.
    fn rejects_affine_model_for_ros_time() {
        let yaml = DOMAIN_CONFIG.replacen("time_scale: device_uptime", "time_scale: ros_time", 1);
        let error = ChronosConfig::from_yaml(&yaml).unwrap_err();
        assert!(
            error
                .to_string()
                .contains("affine_model currently requires device_uptime")
        );
    }

    #[test]
    /// One runtime capability cannot be assigned to two configured domains.
    fn rejects_duplicate_capability_binding() {
        let yaml = DOMAIN_CONFIG.replace(
            "provider_id: front_lidar\n            contract_id: robonix/primitive/lidar/lidar",
            "provider_id: front_camera\n            contract_id: robonix/primitive/camera/rgb",
        );
        let error = ChronosConfig::from_yaml(&yaml).unwrap_err();
        assert!(error.to_string().contains("is assigned more than once"));
    }

    #[test]
    /// Observation cadence and uncertainty must be usable before startup.
    fn rejects_zero_observation_quality() {
        let yaml =
            DOMAIN_CONFIG.replace("sampling_interval_ns: 500000000", "sampling_interval_ns: 0");
        let error = ChronosConfig::from_yaml(&yaml).unwrap_err();
        assert!(error.to_string().contains("sampling interval"));
    }

    #[test]
    /// Strategy-specific fields are rejected instead of silently ignored.
    fn rejects_observation_on_direct_strategy() {
        let yaml = DOMAIN_CONFIG.replace(
            "aligned_to: linux_system",
            "aligned_to: linux_system\n          observation:\n            reference_at: host_receive_time\n            sampling_interval_ns: 1\n            uncertainty_ns: 1",
        );
        let error = ChronosConfig::from_yaml(&yaml).unwrap_err();
        assert!(error.to_string().contains("observation"));
    }

    #[test]
    /// Omitted update policy defaults to step; explicit modes survive domain registration.
    fn parses_model_update_modes() {
        for (setting, expected) in [
            ("", crate::model::ModelUpdatePolicy::Step {}),
            (
                "model_update: {kind: step}",
                crate::model::ModelUpdatePolicy::Step {},
            ),
            (
                "model_update: {kind: slew, max_correction_ppm: 1000}",
                crate::model::ModelUpdatePolicy::Slew {
                    max_correction_ppm: 1000,
                },
            ),
        ] {
            let yaml =
                DOMAIN_CONFIG.replace("policy:\n", &format!("policy:\n            {setting}\n"));
            let config = ChronosConfig::from_yaml(&yaml).unwrap();
            assert_eq!(
                config.chronos.core.affine_model_domains().unwrap()[0]
                    .1
                    .model_update,
                expected
            );
        }
    }

    #[test]
    /// Invalid modes, missing rates and ignored fields fail configuration validation.
    fn rejects_invalid_model_update_modes() {
        for setting in [
            "{kind: slew, max_correction_ppm: 0}",
            "{kind: slew, max_correction_ppm: 1000001}",
            "{kind: slew}",
            "{kind: unknown}",
            "{kind: step, max_correction_ppm: 1000}",
            "{kind: slew, max_correction_ppm: 1000, typo: true}",
        ] {
            let yaml = DOMAIN_CONFIG.replace(
                "policy:\n",
                &format!("policy:\n            model_update: {setting}\n"),
            );
            assert!(
                ChronosConfig::from_yaml(&yaml).is_err(),
                "accepted {setting}"
            );
        }
    }

    /// Register a query adapter for the modeled domain while retaining direct-domain configuration.
    fn query_config() -> String {
        DOMAIN_CONFIG.replace("reference_at: host_receive_time", "reference_at: host_query_midpoint\n            query:\n              adapter: hardware.clock:create\n              tick_ns: 1000\n              options: {serial: 'camera-1'}")
            .replace("wrap_period_ns: null", "wrap_period_ns: 4294967296000")
    }

    #[test]
    /// Query declarations carry raw counter scaling and keep wrap expansion in the core.
    fn parses_query_adapter_configuration() {
        let config = ChronosConfig::from_yaml(&query_config()).unwrap();
        let DomainStrategyConfig::AffineModel { observation, .. } =
            &config.chronos.core.domains["lidar_uptime_clock"].strategy
        else {
            panic!()
        };
        let query = observation.query.as_ref().unwrap();
        assert_eq!(query.adapter, "hardware.clock:create");
        assert_eq!(query.timeout_ns, 50_000_000);
        assert_eq!(query.options["serial"], "camera-1");
        assert_eq!(
            config.chronos.core.affine_model_domains().unwrap()[0]
                .0
                .wrap_period_ns,
            Some(4_294_967_296_000)
        );
    }

    #[test]
    /// Standalone queries need no data capability; driver queries may bind an existing gRPC RPC.
    fn query_bindings_are_optional_and_transport_independent() {
        let mut config = ChronosConfig::from_yaml(&query_config()).unwrap();
        let domain = config
            .chronos
            .core
            .domains
            .get_mut("lidar_uptime_clock")
            .unwrap();
        domain.capabilities[0].transport = CapabilityTransportKind::Grpc;
        domain.capabilities[0].timestamp_field.clear();
        config.chronos.core.validate().unwrap();
        config
            .chronos
            .core
            .domains
            .get_mut("lidar_uptime_clock")
            .unwrap()
            .capabilities
            .clear();
        config.chronos.core.validate().unwrap();
        let yaml = query_config().replace(
            "capabilities:\n          - provider_id: front_lidar\n            contract_id: robonix/primitive/lidar/lidar\n            transport: ros2\n            timestamp_field: header.stamp",
            "capabilities: []",
        );
        assert!(ChronosConfig::from_yaml(&yaml).is_ok());
    }

    #[test]
    /// Query selection never enables ROS implicitly, and the master switch gates both sources.
    fn collection_switches_select_sources_explicitly() {
        let mut config = ChronosConfig::from_yaml(&query_config()).unwrap();
        for (master, ros, expected_query, expected_ros) in [
            (false, false, false, false),
            (false, true, false, false),
            (true, false, true, false),
            (true, true, true, true),
        ] {
            config.chronos.collect_observations = master;
            config.chronos.collect_ros_observations = ros;
            let query = &config.chronos.core.domains["lidar_uptime_clock"];
            assert_eq!(config.chronos.collects_domain(query), expected_query);
            let mut receipt = query.clone();
            if let DomainStrategyConfig::AffineModel { observation, .. } = &mut receipt.strategy {
                observation.query = None;
                observation.reference_at = ObservationReferenceKind::HostReceiveTime;
            }
            assert_eq!(config.chronos.collects_domain(&receipt), expected_ros);
            assert!(
                !config
                    .chronos
                    .collects_domain(&config.chronos.core.domains["front_camera_clock"])
            );
        }
    }

    #[test]
    /// Source mismatches, lossy counter declarations and unsafe wrap cadence fail before startup.
    fn rejects_invalid_query_adapter_configuration() {
        for yaml in [
            query_config().replace("host_query_midpoint", "host_receive_time"),
            DOMAIN_CONFIG.replace("host_receive_time", "host_query_midpoint"),
            query_config().replace("hardware.clock:create", "hardware:create()"),
            query_config().replace("hardware.clock:create", "hardware..clock:create"),
            query_config().replace("tick_ns: 1000", "tick_ns: 0"),
            query_config().replace("tick_ns: 1000", "tick_ns: 9223372036854775808"),
            query_config().replace("tick_ns: 1000", "tick_ns: 999"),
            query_config().replace("fixed_delay_ns: 0", "fixed_delay_ns: 1"),
            query_config().replace(
                "tick_ns: 1000",
                "tick_ns: 1000\n              timeout_ns: 50000001",
            ),
            query_config().replace(
                "tick_ns: 1000",
                "tick_ns: 1000\n              timeout_ns: 0",
            ),
            query_config().replace(
                "sampling_interval_ns: 500000000",
                "sampling_interval_ns: 2147483648000",
            ),
            query_config()
                .replace(
                    "sampling_interval_ns: 500000000",
                    "sampling_interval_ns: 18446744073709551615",
                )
                .replace("max_drift_ppm: 200", "max_drift_ppm: 18446744073709551615"),
        ] {
            assert!(ChronosConfig::from_yaml(&yaml).is_err(), "accepted {yaml}");
        }
    }
}
