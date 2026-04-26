// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx graph` — render the cap topology as PNG/SVG.
//
// Stubbed during the sdk → atlas-client migration (see task #31). Once the
// QueryCapabilities + ConnectCapability flow is wrappable into a topology
// snapshot, port this back. The full SVG renderer + ELK-style layout lived
// in this file before the stub; recover it from git history when reviving.

use std::path::PathBuf;

use anyhow::{bail, Result};
use clap::ValueEnum;

#[derive(Clone, Copy, Debug, Default, ValueEnum)]
#[clap(rename_all = "lower")]
pub enum GraphOutputFormat {
    #[default]
    Png,
    Svg,
}

pub async fn execute(
    _server: &str,
    _output: PathBuf,
    _format: Option<GraphOutputFormat>,
    _test: bool,
) -> Result<()> {
    bail!(
        "rbnx graph is temporarily disabled during the sdk → atlas-client \
         migration. Use `rbnx inspect` once it's ported, or read \
         InspectAtlas's JSON dump directly."
    )
}
