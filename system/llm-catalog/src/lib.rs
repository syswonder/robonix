// SPDX-License-Identifier: MulanPSL-2.0
//! Feature catalog and model resolver for Robonix consumers.
//!
//! The catalog is the single source of truth for what a model can do and how
//! to connect to it.  `ModelCatalog::resolve` filters by feature and request
//! budgets and returns every matching model in preference order; the consumer
//! connects to the first one directly.  This crate does not proxy traffic,
//! track health or manage keys.
use std::collections::{BTreeMap, BTreeSet, HashSet};
use std::path::Path;

use anyhow::{Context, Result, bail};
use serde::{Deserialize, Serialize};

/// One concrete model endpoint a consumer can connect to.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ModelSpec {
    /// Stable identifier used by the catalog and by `resolve` results.
    pub id: String,
    /// Provider family, for diagnostics; the resolver does not match on it.
    pub provider: String,
    /// OpenAI-compatible base URL the consumer connects to.
    pub base_url: String,
    /// Model name sent in the request body.
    pub model: String,
    /// Name of the environment variable holding the API key; never the key.
    /// `None` means the endpoint needs no credential, e.g. local vLLM/Ollama.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub api_key_env: Option<String>,
    /// Feature flags such as `vision`, `tools` or `json_schema`.
    #[serde(default)]
    pub features: BTreeMap<String, bool>,
    /// Context window in tokens; `None` means the catalog sets no limit.
    #[serde(default)]
    pub context: Option<u64>,
    /// Estimated p95 latency in milliseconds, used for ranking and budgets.
    #[serde(default)]
    pub latency_p95_ms: Option<f64>,
    /// Input price in USD per million tokens.
    #[serde(default)]
    pub price_input_per_mtok: f64,
    /// Output price in USD per million tokens.
    #[serde(default)]
    pub price_output_per_mtok: f64,
    /// Wire protocol the consumer must speak; currently `openai-chat`.
    #[serde(default = "default_protocol")]
    pub protocol: String,
    /// Set by [`ModelCatalog::resolve`] when the request asked for `json_schema`
    /// but only a `json_object` model fit.  Never read from the catalog file.
    #[serde(skip)]
    pub downgraded: bool,
}

fn default_protocol() -> String {
    "openai-chat".to_owned()
}

impl ModelSpec {
    #[must_use]
    pub fn supports(&self, feature: &str) -> bool {
        self.features.get(feature).copied().unwrap_or(false)
    }
}

/// A catalog file: a non-empty list of model endpoints.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ModelCatalog {
    /// Every model endpoint the deployment may use; ids are unique.
    pub models: Vec<ModelSpec>,
}

/// What a request needs.  Features are hard requirements; budgets are
/// optional.  An unknown latency is rejected when a latency budget is set.
#[derive(Debug, Clone)]
pub struct Requirements {
    /// Features the model must support; all of them are required.
    pub features: Vec<String>,
    /// Estimated input tokens for the request.
    pub input_tokens: u64,
    /// Maximum tokens the model may generate.
    pub max_output_tokens: u64,
    /// Optional p95 latency budget; unknown latency is rejected.
    pub max_latency_ms: Option<f64>,
    /// Optional per-call cost budget in USD.
    pub max_cost_usd_per_call: Option<f64>,
    /// Whether a `json_schema` request may fall back to `json_object`.
    pub allow_downgrade: bool,
}

impl Default for Requirements {
    fn default() -> Self {
        Self {
            features: Vec::new(),
            input_tokens: 0,
            max_output_tokens: 0,
            max_latency_ms: None,
            max_cost_usd_per_call: None,
            allow_downgrade: true,
        }
    }
}

impl Requirements {
    /// Build request requirements from the `ROBONIX_LLM_*` environment.
    /// `default_features` is used when `ROBONIX_LLM_FEATURES` is absent.
    #[must_use]
    pub fn from_env(default_features: &str) -> Self {
        let features =
            std::env::var("ROBONIX_LLM_FEATURES").unwrap_or_else(|_| default_features.to_owned());
        Self {
            features: features
                .split(',')
                .map(str::trim)
                .filter(|feature| !feature.is_empty())
                .map(ToOwned::to_owned)
                .collect(),
            input_tokens: env_u64("ROBONIX_LLM_INPUT_TOKENS").unwrap_or(0),
            max_output_tokens: env_u64("ROBONIX_LLM_MAX_OUTPUT_TOKENS").unwrap_or(0),
            max_latency_ms: env_f64("ROBONIX_LLM_MAX_LATENCY_MS"),
            max_cost_usd_per_call: env_f64("ROBONIX_LLM_MAX_COST_USD_PER_CALL"),
            allow_downgrade: env_bool("ROBONIX_LLM_ALLOW_DOWNGRADE").unwrap_or(true),
        }
    }
}

fn env_u64(name: &str) -> Option<u64> {
    std::env::var(name).ok()?.parse().ok()
}

fn env_f64(name: &str) -> Option<f64> {
    std::env::var(name).ok()?.parse().ok()
}

fn env_bool(name: &str) -> Option<bool> {
    match std::env::var(name).ok()?.to_ascii_lowercase().as_str() {
        "1" | "true" | "yes" | "on" => Some(true),
        "0" | "false" | "no" | "off" => Some(false),
        _ => None,
    }
}

impl ModelCatalog {
    /// Load a JSON or YAML catalog, chosen by file extension.
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("read LLM catalog {}", path.display()))?;
        let is_json = path
            .extension()
            .is_some_and(|extension| extension.eq_ignore_ascii_case("json"));
        let mut catalog: Self = if is_json {
            serde_json::from_str(&text)
                .with_context(|| format!("parse JSON catalog {}", path.display()))?
        } else {
            // Typed YAML deserialization otherwise coerces booleans into strings.
            serde_yaml::from_str::<serde_yaml::Value>(&text)
                .and_then(serde_yaml::from_value::<Self>)
                .with_context(|| format!("parse YAML catalog {}", path.display()))?
        };
        catalog.validate()?;
        Ok(catalog)
    }

    /// Return the model with `id`, or an error when the catalog has no such id.
    pub fn model(&self, id: &str) -> Result<&ModelSpec> {
        self.models
            .iter()
            .find(|model| model.id == id)
            .with_context(|| format!("unknown model {id:?}"))
    }

    /// Return matching models, best first: latency, then token price, then id.
    ///
    /// When `json_schema` is required but no schema-capable model fits the
    /// request budgets, `allow_downgrade` lets a `json_object` model stand in;
    /// every returned model then carries `downgraded = true`.
    #[must_use]
    pub fn resolve(&self, requirements: &Requirements) -> Vec<ModelSpec> {
        let mut required: BTreeSet<String> = requirements.features.iter().cloned().collect();
        let mut downgraded = false;

        if required.contains("json_schema")
            && requirements.allow_downgrade
            && self.matching(&required, requirements).is_empty()
        {
            let mut relaxed = required.clone();
            relaxed.remove("json_schema");
            relaxed.insert("json_object".to_owned());
            if !self.matching(&relaxed, requirements).is_empty() {
                required = relaxed;
                downgraded = true;
            }
        }

        let mut matches = self.matching(&required, requirements);
        matches.sort_by(|left, right| {
            let left_latency = left.latency_p95_ms.unwrap_or(f64::INFINITY);
            let right_latency = right.latency_p95_ms.unwrap_or(f64::INFINITY);
            let left_price = left.price_input_per_mtok + left.price_output_per_mtok;
            let right_price = right.price_input_per_mtok + right.price_output_per_mtok;
            left_latency
                .total_cmp(&right_latency)
                .then(left_price.total_cmp(&right_price))
                .then_with(|| left.id.cmp(&right.id))
        });
        for model in &mut matches {
            model.downgraded = downgraded;
        }
        matches
    }

    fn matching(&self, required: &BTreeSet<String>, req: &Requirements) -> Vec<ModelSpec> {
        self.models
            .iter()
            .filter(|model| fits(model, required, req))
            .cloned()
            .collect()
    }

    fn validate(&mut self) -> Result<()> {
        if self.models.is_empty() {
            bail!("catalog.models must be a non-empty list");
        }
        let mut seen = HashSet::new();
        for model in &mut self.models {
            for (field, value) in [
                ("id", &mut model.id),
                ("provider", &mut model.provider),
                ("base_url", &mut model.base_url),
                ("model", &mut model.model),
                ("protocol", &mut model.protocol),
            ] {
                *value = value.trim().to_owned();
                if value.is_empty() {
                    bail!("model.{field} must not be empty");
                }
            }
            model.base_url = model.base_url.trim_end_matches('/').to_owned();
            if model.base_url.is_empty() {
                bail!("model.base_url must not be empty");
            }
            if let Some(name) = &model.api_key_env {
                let valid_start = name
                    .as_bytes()
                    .first()
                    .is_some_and(|byte| byte.is_ascii_alphabetic() || *byte == b'_');
                if !valid_start
                    || !name.bytes().all(|byte| byte.is_ascii_alphanumeric() || byte == b'_')
                {
                    bail!("model.api_key_env must be a valid environment variable name");
                }
            }
            if !seen.insert(model.id.clone()) {
                bail!("duplicate model id {:?}", model.id);
            }
            if model.context == Some(0) {
                bail!("model {:?}.context must be positive", model.id);
            }
            if [model.price_input_per_mtok, model.price_output_per_mtok]
                .iter()
                .any(|price| !price.is_finite() || *price < 0.0)
            {
                bail!("model {:?} prices must be finite and non-negative", model.id);
            }
            if model
                .latency_p95_ms
                .is_some_and(|latency| !latency.is_finite() || latency < 0.0)
            {
                bail!("model {:?}.latency_p95_ms must be finite and non-negative", model.id);
            }
        }
        Ok(())
    }
}

fn fits(model: &ModelSpec, required: &BTreeSet<String>, req: &Requirements) -> bool {
    if !required.iter().all(|feature| model.supports(feature)) {
        return false;
    }
    let wanted_tokens = req.input_tokens.saturating_add(req.max_output_tokens);
    if model.context.is_some_and(|context| wanted_tokens > context) {
        return false;
    }
    if let Some(budget) = req.max_latency_ms {
        match model.latency_p95_ms {
            Some(latency) if latency <= budget => {}
            _ => return false,
        }
    }
    if let Some(budget) = req.max_cost_usd_per_call
        && estimated_cost(model, req) > budget
    {
        return false;
    }
    true
}

fn estimated_cost(model: &ModelSpec, req: &Requirements) -> f64 {
    req.input_tokens as f64 / 1_000_000.0 * model.price_input_per_mtok
        + req.max_output_tokens as f64 / 1_000_000.0 * model.price_output_per_mtok
}

/// Resolve one target from `ROBONIX_LLM_CATALOG`, or `None` when unset.
///
/// Load the catalog named by `ROBONIX_LLM_CATALOG`, or `None` when unset.
pub fn catalog_from_env() -> Result<Option<ModelCatalog>> {
    let path = std::env::var("ROBONIX_LLM_CATALOG").unwrap_or_default();
    let path = path.trim();
    if path.is_empty() {
        return Ok(None);
    }
    ModelCatalog::load(Path::new(path)).map(Some)
}

/// A target with no `api_key_env` needs no credential and stays eligible. A
/// target whose `api_key_env` is set but missing from the environment is
/// skipped; if no target is usable, the error names the variables to set. The
/// catalog is read on each call; callers that resolve once keep the result.
pub fn resolve_from_env(requirements: &Requirements) -> Result<Option<ModelSpec>> {
    let path = std::env::var("ROBONIX_LLM_CATALOG").unwrap_or_default();
    let path = path.trim();
    if path.is_empty() {
        return Ok(None);
    }
    let catalog = ModelCatalog::load(Path::new(path))?;
    let available = ModelCatalog {
        models: catalog
            .models
            .iter()
            .filter(|model| model.api_key_env.is_none() || !api_key_for(model).is_empty())
            .cloned()
            .collect(),
    };
    if let Some(target) = available.resolve(requirements).into_iter().next() {
        return Ok(Some(target));
    }
    let targets = catalog.resolve(requirements);
    if targets.is_empty() {
        bail!(
            "no model in {path} satisfies {}",
            requirements.features.join(",")
        );
    }
    let mut missing: Vec<&str> = targets
        .iter()
        .filter_map(|target| target.api_key_env.as_deref())
        .collect();
    missing.sort_unstable();
    missing.dedup();
    bail!(
        "no usable model in {path} for {}; set one of {}",
        requirements.features.join(","),
        missing.join(", ")
    )
}

/// The key the consumer must send, or an empty string when unset/keyless.
#[must_use]
pub fn api_key_for(target: &ModelSpec) -> String {
    target
        .api_key_env
        .as_deref()
        .and_then(|name| std::env::var(name).ok())
        .unwrap_or_default()
}
