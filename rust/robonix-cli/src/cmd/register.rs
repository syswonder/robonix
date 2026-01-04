// SPDX-License-Identifier: MulanPSL-2.0
// Register Command Module
//
// Register command implementation for robonix-cli

use crate::{Config, PackageRegistrar};
use anyhow::Result;
use std::path::PathBuf;

pub async fn execute(config: Config, recipe: PathBuf) -> Result<()> {
    let registrar = PackageRegistrar::new(config)?;
    registrar.register_from_recipe(&recipe).await?;
    Ok(())
}
