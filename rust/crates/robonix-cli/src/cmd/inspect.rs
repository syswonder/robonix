// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx caps / describe / tools / channels / inspect` — atlas introspection.
//
// All these are read-only views over `AtlasClient::query_capabilities` and
// `AtlasClient::inner().inspect_atlas`. No state mutation, no Connect.

use anyhow::{Context, Result};
use colored::*;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use serde_json::Value;

fn transport_name(t: i32) -> &'static str {
    match atlas_pb::Transport::try_from(t).unwrap_or(atlas_pb::Transport::Unspecified) {
        atlas_pb::Transport::Grpc => "grpc",
        atlas_pb::Transport::Ros2 => "ros2",
        atlas_pb::Transport::Mcp => "mcp",
        atlas_pb::Transport::Unspecified => "?",
    }
}

fn state_name(s: i32) -> &'static str {
    match atlas_pb::CapabilityState::try_from(s)
        .unwrap_or(atlas_pb::CapabilityState::StateUnspecified)
    {
        atlas_pb::CapabilityState::StateRegistered => "REGISTERED",
        atlas_pb::CapabilityState::StateInitialized => "INITIALIZED",
        atlas_pb::CapabilityState::StateOnline => "ONLINE",
        atlas_pb::CapabilityState::StateOffline => "OFFLINE",
        atlas_pb::CapabilityState::StateError => "ERROR",
        atlas_pb::CapabilityState::StateUnspecified => "?",
    }
}

/// Color the [STATE] tag the same way every printer does it. Picked to
/// match the boot-log feel: green=running healthy, yellow=came up but
/// nothing's driving it yet, red=problem, dim=quiescent.
fn state_tag(s: i32) -> colored::ColoredString {
    let label = format!("[{}]", state_name(s));
    match atlas_pb::CapabilityState::try_from(s)
        .unwrap_or(atlas_pb::CapabilityState::StateUnspecified)
    {
        atlas_pb::CapabilityState::StateOnline => label.green().bold(),
        atlas_pb::CapabilityState::StateInitialized => label.yellow(),
        atlas_pb::CapabilityState::StateRegistered => label.blue(),
        atlas_pb::CapabilityState::StateOffline => label.dimmed(),
        atlas_pb::CapabilityState::StateError => label.red().bold(),
        atlas_pb::CapabilityState::StateUnspecified => label.dimmed(),
    }
}

async fn connect(endpoint: &str) -> Result<AtlasClient> {
    AtlasClient::connect(endpoint)
        .await
        .with_context(|| format!("connect to atlas at '{endpoint}'"))
}

pub async fn caps(endpoint: &str, json: bool, verbose: bool) -> Result<()> {
    let mut atlas = connect(endpoint).await?;
    let records = atlas
        .query_capabilities("", "", atlas_pb::Transport::Unspecified)
        .await?;
    if json {
        let serialised: Vec<_> = records
            .iter()
            .map(|r| {
                serde_json::json!({
                    "capability_id":  r.capability_id,
                    "namespace":      r.namespace,
                    "state":          state_name(r.state),
                    "state_detail":   r.state_detail,
                    "interfaces":     r.interfaces.iter().map(|i| serde_json::json!({
                        "contract_id": i.contract_id,
                        "transport":   transport_name(i.transport),
                    })).collect::<Vec<_>>(),
                })
            })
            .collect();
        println!("{}", serde_json::to_string_pretty(&serialised)?);
        return Ok(());
    }

    if records.is_empty() {
        println!("{} no capabilities registered", "[caps]".yellow().bold());
        return Ok(());
    }
    // Default: one row per cap, no interfaces. -v expands the interface
    // list (lspci -tv style — quick scan vs full dump).
    for rec in &records {
        let detail = if rec.state_detail.is_empty() {
            String::new()
        } else {
            format!(" — {}", rec.state_detail)
        };
        let iface_count_hint = if verbose {
            String::new()
        } else {
            format!(" ({} ifaces)", rec.interfaces.len()).dimmed().to_string()
        };
        println!(
            "{} {} {} {}{}{}",
            "●".green(),
            rec.capability_id.bold(),
            state_tag(rec.state),
            rec.namespace.dimmed(),
            iface_count_hint,
            detail.dimmed()
        );
        if verbose {
            for iface in &rec.interfaces {
                println!(
                    "    {} {} {}",
                    "└─".dimmed(),
                    iface.contract_id,
                    format!("({})", transport_name(iface.transport)).dimmed()
                );
            }
        }
    }
    if !verbose {
        println!("\n{} pass {} for the per-cap interface list",
                 "tip:".dimmed(), "-v".bold());
    }
    Ok(())
}

pub async fn describe(endpoint: &str, cap_id: Option<&str>, json: bool) -> Result<()> {
    let mut atlas = connect(endpoint).await?;
    let cap_filter = cap_id.unwrap_or("");
    let records = atlas
        .query_capabilities(cap_filter, "", atlas_pb::Transport::Unspecified)
        .await?;
    if records.is_empty() {
        println!("{} no matching capabilities", "[describe]".yellow().bold());
        return Ok(());
    }
    for rec in &records {
        let md = atlas
            .query_capability_md(&rec.capability_id)
            .await
            .unwrap_or_default();
        if json {
            let value = serde_json::json!({
                "capability_id":   rec.capability_id,
                "namespace":       rec.namespace,
                "state":           state_name(rec.state),
                "interfaces":      rec.interfaces.iter().map(|i| serde_json::json!({
                    "contract_id": i.contract_id,
                    "transport":   transport_name(i.transport),
                })).collect::<Vec<_>>(),
                "capability_md":   md,
            });
            println!("{}", serde_json::to_string_pretty(&value)?);
        } else {
            println!(
                "{} {} {}",
                "●".green(),
                rec.capability_id.bold(),
                state_tag(rec.state)
            );
            for iface in &rec.interfaces {
                println!(
                    "    {} {} ({})",
                    "└─".dimmed(),
                    iface.contract_id,
                    transport_name(iface.transport)
                );
            }
            if !md.is_empty() {
                println!("\n{}", md);
            }
        }
    }
    Ok(())
}

pub async fn tools(endpoint: &str, json: bool) -> Result<()> {
    // "Tools" in Robonix-speak = MCP-transport interfaces (LLM-callable caps).
    let mut atlas = connect(endpoint).await?;
    let records = atlas
        .query_capabilities("", "", atlas_pb::Transport::Mcp)
        .await?;
    let mut entries: Vec<(String, String, String, String)> = Vec::new();
    for rec in &records {
        for iface in &rec.interfaces {
            if iface.transport != atlas_pb::Transport::Mcp as i32 {
                continue;
            }
            let (description, schema) = match iface.params.as_ref().and_then(|p| p.kind.as_ref()) {
                Some(atlas_pb::transport_params::Kind::Mcp(m)) => {
                    (m.description.clone(), m.input_schema_json.clone())
                }
                _ => (String::new(), String::new()),
            };
            entries.push((
                rec.capability_id.clone(),
                iface.contract_id.clone(),
                description,
                schema,
            ));
        }
    }
    if json {
        let serialised: Vec<_> = entries
            .iter()
            .map(|(cap, c, d, s)| {
                serde_json::json!({
                    "cap_id":            cap,
                    "contract_id":       c,
                    "description":       d,
                    "input_schema_json": s,
                })
            })
            .collect();
        println!("{}", serde_json::to_string_pretty(&serialised)?);
        return Ok(());
    }
    if entries.is_empty() {
        println!("{} no MCP tools registered", "[tools]".yellow().bold());
        return Ok(());
    }
    for (cap, c, d, _s) in &entries {
        let leaf = c.rsplit_once('/').map(|(_, leaf)| leaf).unwrap_or(c);
        println!(
            "{} {}  {}",
            "●".green(),
            leaf.bold(),
            format!("[{}]", c).dimmed()
        );
        println!("    cap   : {}", cap.dimmed());
        if !d.is_empty() {
            println!("    desc  : {}", d);
        }
    }
    Ok(())
}

pub async fn channels(endpoint: &str) -> Result<()> {
    let atlas = connect(endpoint).await?;
    let raw = atlas
        .inner()
        .inspect_atlas(atlas_pb::InspectAtlasRequest {})
        .await
        .context("InspectAtlas RPC")?
        .into_inner()
        .json;
    let v: Value = serde_json::from_str(&raw).context("parse inspect json")?;
    let channels = v
        .get("channels")
        .and_then(|c| c.as_object())
        .cloned()
        .unwrap_or_default();
    if channels.is_empty() {
        println!("{} no active channels", "[channels]".yellow().bold());
        return Ok(());
    }
    for (id, ch) in channels.iter() {
        let consumer = ch
            .get("consumer_id")
            .and_then(|x| x.as_str())
            .unwrap_or("?");
        let provider = ch
            .get("provider_cap_id")
            .and_then(|x| x.as_str())
            .unwrap_or("?");
        let contract = ch
            .get("contract_id")
            .and_then(|x| x.as_str())
            .unwrap_or("?");
        let transport = ch.get("transport").and_then(|x| x.as_str()).unwrap_or("?");
        let endpoint = ch.get("endpoint").and_then(|x| x.as_str()).unwrap_or("?");
        println!("{} {}", "●".green(), id.bold());
        println!("    consumer : {}", consumer);
        println!(
            "    provider : {} ({} via {})",
            provider, contract, transport
        );
        println!("    endpoint : {}", endpoint.dimmed());
    }
    Ok(())
}

pub async fn inspect(endpoint: &str) -> Result<()> {
    let atlas = connect(endpoint).await?;
    let raw = atlas
        .inner()
        .inspect_atlas(atlas_pb::InspectAtlasRequest {})
        .await
        .context("InspectAtlas RPC")?
        .into_inner()
        .json;
    println!("{raw}");
    Ok(())
}
