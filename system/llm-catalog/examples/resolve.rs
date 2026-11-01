// SPDX-License-Identifier: MulanPSL-2.0
//! Print the models that satisfy a feature request.
//!
//! cargo run -p robonix-llm-catalog --example resolve -- \
//!   pylib/robonix-api/robonix_api/llm/catalog.example.yaml vision,tools

use std::path::PathBuf;

use robonix_llm_catalog::{ModelCatalog, Requirements};

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let catalog_path = args
        .next()
        .unwrap_or_else(|| "pylib/robonix-api/robonix_api/llm/catalog.example.yaml".to_owned());
    let features = args.next().unwrap_or_default();
    let catalog = ModelCatalog::load(&PathBuf::from(catalog_path))?;
    let requirements = Requirements {
        features: features
            .split(',')
            .map(str::trim)
            .filter(|feature| !feature.is_empty())
            .map(ToOwned::to_owned)
            .collect(),
    };

    for target in catalog.resolve(&requirements) {
        println!(
            "{}  {}  {}  env={}",
            target.id,
            target.base_url,
            target.model,
            target.api_key_env.as_deref().unwrap_or("<none>")
        );
    }
    Ok(())
}
