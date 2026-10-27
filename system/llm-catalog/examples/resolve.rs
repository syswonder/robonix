// SPDX-License-Identifier: MulanPSL-2.0
//! Print the models that satisfy a capability request.
//!
//! cargo run -p robonix-llm-catalog --example resolve -- \
//!   pylib/robonix-api/robonix_api/llm/catalog.example.yaml vision,tools 1000 256

use std::path::PathBuf;

use robonix_llm_catalog::{Catalog, Requirements};

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let catalog_path = args
        .next()
        .unwrap_or_else(|| "pylib/robonix-api/robonix_api/llm/catalog.example.yaml".to_owned());
    let capabilities = args.next().unwrap_or_default();
    let input_tokens = args
        .next()
        .and_then(|value| value.parse().ok())
        .unwrap_or(0);
    let max_output_tokens = args
        .next()
        .and_then(|value| value.parse().ok())
        .unwrap_or(0);

    let catalog = Catalog::load(&PathBuf::from(catalog_path))?;
    let requirements = Requirements {
        capabilities: capabilities
            .split(',')
            .map(str::trim)
            .filter(|capability| !capability.is_empty())
            .map(ToOwned::to_owned)
            .collect(),
        input_tokens,
        max_output_tokens,
        ..Requirements::default()
    };

    for target in catalog.resolve(&requirements) {
        println!(
            "{}  {}  {}  env={}  downgraded={}",
            target.id, target.base_url, target.model, target.api_key_env, target.downgraded
        );
    }
    Ok(())
}
