// SPDX-License-Identifier: MulanPSL-2.0
//! Capability catalog and model resolver for Robonix consumers.
//!
//! The catalog is the single source of truth for what a model can do and how
//! to connect to it.  `Catalog::resolve` filters by capability and request
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
    pub api_key_env: String,
    /// Capability flags such as `vision`, `tools` or `json_schema`.
    #[serde(default)]
    pub capabilities: BTreeMap<String, bool>,
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
    /// Set by [`Catalog::resolve`] when the request asked for `json_schema`
    /// but only a `json_object` model fit.  Never read from the catalog file.
    #[serde(skip)]
    pub downgraded: bool,
}

fn default_protocol() -> String {
    "openai-chat".to_owned()
}

impl ModelSpec {
    #[must_use]
    pub fn supports(&self, capability: &str) -> bool {
        self.capabilities.get(capability).copied().unwrap_or(false)
    }
}

/// A catalog file: a non-empty list of model endpoints.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Catalog {
    /// Every model endpoint the deployment may use; ids are unique.
    pub models: Vec<ModelSpec>,
}

/// What a request needs.  Capabilities are hard requirements; budgets are
/// optional.  An unknown latency is rejected when a latency budget is set.
#[derive(Debug, Clone)]
pub struct Requirements {
    /// Capabilities the model must support; all of them are required.
    pub capabilities: Vec<String>,
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
            capabilities: Vec::new(),
            input_tokens: 0,
            max_output_tokens: 0,
            max_latency_ms: None,
            max_cost_usd_per_call: None,
            allow_downgrade: true,
        }
    }
}

impl Catalog {
    /// Load a JSON or YAML catalog, chosen by file extension.
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("read LLM catalog {}", path.display()))?;
        let is_json = path
            .extension()
            .is_some_and(|extension| extension.eq_ignore_ascii_case("json"));
        let catalog: Self = if is_json {
            serde_json::from_str(&text)
                .with_context(|| format!("parse JSON catalog {}", path.display()))?
        } else {
            serde_yaml::from_str(&text)
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
        let mut required: BTreeSet<String> = requirements.capabilities.iter().cloned().collect();
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

    fn validate(&self) -> Result<()> {
        if self.models.is_empty() {
            bail!("catalog.models must be a non-empty list");
        }
        let mut seen = HashSet::new();
        for model in &self.models {
            for (field, value) in [
                ("id", &model.id),
                ("provider", &model.provider),
                ("base_url", &model.base_url),
                ("model", &model.model),
                ("api_key_env", &model.api_key_env),
                ("protocol", &model.protocol),
            ] {
                if value.trim().is_empty() {
                    bail!("model.{field} must not be empty");
                }
            }
            if !seen.insert(model.id.as_str()) {
                bail!("duplicate model id {:?}", model.id);
            }
            if model.context == Some(0) {
                bail!("model {:?}.context must be positive", model.id);
            }
            if model.price_input_per_mtok < 0.0 || model.price_output_per_mtok < 0.0 {
                bail!("model {:?} prices must not be negative", model.id);
            }
            if model.latency_p95_ms.is_some_and(|latency| latency < 0.0) {
                bail!("model {:?}.latency_p95_ms must not be negative", model.id);
            }
        }
        Ok(())
    }
}

fn fits(model: &ModelSpec, required: &BTreeSet<String>, req: &Requirements) -> bool {
    if !required.iter().all(|capability| model.supports(capability)) {
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
/// A target whose `api_key_env` is present wins over an earlier target with a
/// missing key; if no key is set the first target is returned so the caller's
/// existing "no key -> inactive" path still applies. The catalog is read on
/// each call; callers that resolve once keep the result.
pub fn resolve_from_env(requirements: &Requirements) -> Result<Option<ModelSpec>> {
    let path = std::env::var("ROBONIX_LLM_CATALOG").unwrap_or_default();
    let path = path.trim();
    if path.is_empty() {
        return Ok(None);
    }
    let catalog = Catalog::load(Path::new(path))?;
    let targets = catalog.resolve(requirements);
    if targets.is_empty() {
        bail!(
            "no model in {path} satisfies {}",
            requirements.capabilities.join(",")
        );
    }
    for target in &targets {
        if std::env::var(&target.api_key_env).is_ok_and(|value| !value.is_empty()) {
            return Ok(Some(target.clone()));
        }
    }
    Ok(targets.into_iter().next())
}

/// The key the consumer must send, or an empty string when unset.
#[must_use]
pub fn api_key_for(target: &ModelSpec) -> String {
    std::env::var(&target.api_key_env).unwrap_or_default()
}
