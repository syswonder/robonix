// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx nodes / describe / tools / channels / inspect` — runtime introspection.
//
// Stubbed during the sdk → atlas-client migration (see task #31). The shape
// these commands need post-migration is roughly:
//
//   nodes      → AtlasClient::query_capabilities("", "", Unspecified)
//   describe   → query_capabilities + query_capability_md
//   tools      → SystemExecutorListTools.Call (via ConnectCapability)
//   channels   → InspectAtlas (channels live in the JSON dump now)
//   inspect    → InspectAtlas (raw JSON)

use anyhow::{bail, Result};

const MIGRATION_NOTE: &str = "this command is temporarily disabled during the \
                              sdk → atlas-client migration. Use `atlas` gRPC \
                              directly (QueryCapabilities / InspectAtlas) for \
                              now.";

pub async fn nodes(
    _endpoint: &str,
    _distro: Option<&str>,
    _container: Option<&str>,
    _json: bool,
) -> Result<()> {
    bail!("rbnx nodes: {MIGRATION_NOTE}")
}

pub async fn describe(_endpoint: &str, _node_id: Option<&str>, _json: bool) -> Result<()> {
    bail!("rbnx describe: {MIGRATION_NOTE}")
}

pub async fn tools(_endpoint: &str, _json: bool) -> Result<()> {
    bail!("rbnx tools: {MIGRATION_NOTE}")
}

pub async fn channels(_endpoint: &str) -> Result<()> {
    bail!("rbnx channels: {MIGRATION_NOTE}")
}

pub async fn inspect(_endpoint: &str) -> Result<()> {
    bail!("rbnx inspect: {MIGRATION_NOTE}")
}
