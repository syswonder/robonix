// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx chat` — interactive TUI agent client.
//
// Stubbed during the sdk → atlas-client migration (see task #31). Once the
// pilot's SystemPilot Stream RPC is reachable via robonix_atlas::client,
// this should be ported to `connect_to_capability(..., "robonix/system/pilot")`
// and then dial that channel. The previous TUI lives in git history (last
// working at commit 7f3f63f's parent on dev-packaging).

use anyhow::{Result, bail};

pub async fn execute(_server: &str) -> Result<()> {
    bail!(
        "rbnx chat is temporarily disabled during the sdk → atlas-client \
         migration. Talk to pilot directly over its `robonix/system/pilot` \
         gRPC contract for now (see crates/robonix-pilot)."
    )
}
