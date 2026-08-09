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
    match atlas_pb::LifecycleState::try_from(s)
        .unwrap_or(atlas_pb::LifecycleState::StateUnspecified)
    {
        atlas_pb::LifecycleState::StateRegistered => "REGISTERED",
        atlas_pb::LifecycleState::StateInactive => "INACTIVE",
        atlas_pb::LifecycleState::StateActive => "ACTIVE",
        atlas_pb::LifecycleState::StateError => "ERROR",
        atlas_pb::LifecycleState::StateTerminated => "TERMINATED",
        atlas_pb::LifecycleState::StateUnspecified => "?",
    }
}

/// Color the `[STATE]` tag the same way every printer does it. Picked to
/// match the boot-log feel: green=running healthy, yellow=came up but
/// nothing's driving it yet, red=problem, dim=quiescent.
fn state_tag(s: i32) -> colored::ColoredString {
    let label = format!("[{}]", state_name(s));
    match atlas_pb::LifecycleState::try_from(s)
        .unwrap_or(atlas_pb::LifecycleState::StateUnspecified)
    {
        atlas_pb::LifecycleState::StateActive => label.green().bold(),
        atlas_pb::LifecycleState::StateInactive => label.yellow(),
        atlas_pb::LifecycleState::StateRegistered => label.blue(),
        atlas_pb::LifecycleState::StateTerminated => label.dimmed(),
        atlas_pb::LifecycleState::StateError => label.red().bold(),
        atlas_pb::LifecycleState::StateUnspecified => label.dimmed(),
    }
}

async fn connect(endpoint: &str) -> Result<AtlasClient> {
    AtlasClient::connect(endpoint)
        .await
        .with_context(|| format!("connect to atlas at '{endpoint}'"))
}

pub async fn providers(endpoint: &str, json: bool, verbose: bool) -> Result<()> {
    let mut atlas = connect(endpoint).await?;
    let providers = atlas
        .query_capabilities("", "", atlas_pb::Transport::Unspecified)
        .await?;
    if json {
        let serialised: Vec<_> = providers
            .iter()
            .map(|r| {
                serde_json::json!({
                    "provider_id":  r.id,
                    "namespace":      r.namespace,
                    "state":          state_name(r.state),
                    "state_detail":   r.state_detail,
                    "capabilities":     r.capabilities.iter().map(|i| serde_json::json!({
                        "contract_id": i.contract_id,
                        "transport":   transport_name(i.transport),
                        "namespace_mismatch": i.namespace_mismatch,
                    })).collect::<Vec<_>>(),
                })
            })
            .collect();
        println!("{}", serde_json::to_string_pretty(&serialised)?);
        return Ok(());
    }

    if providers.is_empty() {
        println!("{} no providers registered", "[providers]".yellow().bold());
        return Ok(());
    }
    // Default: one row per provider, no capabilities. -v expands the
    // capability list (lspci -tv style — quick scan vs full dump).
    for provider in &providers {
        let detail = if provider.state_detail.is_empty() {
            String::new()
        } else {
            format!(" — {}", provider.state_detail)
        };
        let cap_count_hint = if verbose {
            String::new()
        } else {
            format!(" ({} caps)", provider.capabilities.len())
                .dimmed()
                .to_string()
        };
        let namespace_hint = if provider
            .capabilities
            .iter()
            .any(|cap| cap.namespace_mismatch)
        {
            " [namespace mismatch]".yellow().to_string()
        } else {
            String::new()
        };
        println!(
            "{} {} {} {}{}{}{}",
            "●".green(),
            provider.id.bold(),
            state_tag(provider.state),
            provider.namespace.dimmed(),
            cap_count_hint,
            namespace_hint,
            detail.dimmed()
        );
        if verbose {
            for cap in &provider.capabilities {
                let mismatch = if cap.namespace_mismatch {
                    " [namespace mismatch]".yellow().to_string()
                } else {
                    String::new()
                };
                println!(
                    "    {} {} {}{}",
                    "└─".dimmed(),
                    cap.contract_id,
                    format!("({})", transport_name(cap.transport)).dimmed(),
                    mismatch
                );
            }
        }
    }
    if !verbose {
        println!(
            "\n{} pass {} for the per-provider capability list",
            "tip:".dimmed(),
            "-v".bold()
        );
    }
    Ok(())
}

pub async fn describe(endpoint: &str, provider_id: Option<&str>, json: bool) -> Result<()> {
    let mut atlas = connect(endpoint).await?;
    let cap_filter = provider_id.unwrap_or("");
    let providers = atlas
        .query_capabilities(cap_filter, "", atlas_pb::Transport::Unspecified)
        .await?;
    if providers.is_empty() {
        println!("{} no matching providers", "[describe]".yellow().bold());
        return Ok(());
    }
    for provider in &providers {
        // Atlas only stores the CAPABILITY.md path; consumers read the
        // file off the local filesystem themselves.
        let md = if provider.capability_md_path.is_empty() {
            String::new()
        } else {
            std::fs::read_to_string(&provider.capability_md_path).unwrap_or_default()
        };
        if json {
            let value = serde_json::json!({
                "provider_id":   provider.id,
                "namespace":       provider.namespace,
                "state":           state_name(provider.state),
                "capabilities":      provider.capabilities.iter().map(|i| serde_json::json!({
                    "contract_id": i.contract_id,
                    "transport":   transport_name(i.transport),
                    "namespace_mismatch": i.namespace_mismatch,
                })).collect::<Vec<_>>(),
                "capability_md":   md,
            });
            println!("{}", serde_json::to_string_pretty(&value)?);
        } else {
            println!(
                "{} {} {}",
                "●".green(),
                provider.id.bold(),
                state_tag(provider.state)
            );
            for cap in &provider.capabilities {
                let mismatch = if cap.namespace_mismatch {
                    " [namespace mismatch]".yellow().to_string()
                } else {
                    String::new()
                };
                println!(
                    "    {} {} ({}){}",
                    "└─".dimmed(),
                    cap.contract_id,
                    transport_name(cap.transport),
                    mismatch
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
    // "Tools" in Robonix-speak = MCP-transport capabilities (LLM-callable providers).
    let mut atlas = connect(endpoint).await?;
    let providers = atlas
        .query_capabilities("", "", atlas_pb::Transport::Mcp)
        .await?;
    let mut entries: Vec<(String, String, String, String)> = Vec::new();
    for provider in &providers {
        for cap in &provider.capabilities {
            if cap.transport != atlas_pb::Transport::Mcp as i32 {
                continue;
            }
            let schema = match cap.params.as_ref().and_then(|p| p.kind.as_ref()) {
                Some(atlas_pb::transport_params::Kind::Mcp(m)) => m.input_schema_json.clone(),
                _ => String::new(),
            };
            let description = cap.description.clone();
            entries.push((
                provider.id.clone(),
                cap.contract_id.clone(),
                description,
                schema,
            ));
        }
    }
    if json {
        let serialised: Vec<_> = entries
            .iter()
            .map(|(provider, c, d, s)| {
                serde_json::json!({
                    "provider_id":            provider,
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
    for (provider, c, d, _s) in &entries {
        let leaf = c.rsplit_once('/').map(|(_, leaf)| leaf).unwrap_or(c);
        println!(
            "{} {}  {}",
            "●".green(),
            leaf.bold(),
            format!("[{}]", c).dimmed()
        );
        println!("    provider   : {}", provider.dimmed());
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
            .get("provider_id")
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

pub async fn contracts(
    endpoint: &str,
    prefix: Option<&str>,
    json: bool,
    verbose: bool,
) -> Result<()> {
    let atlas = connect(endpoint).await?;
    let resp = atlas
        .inner()
        .list_contracts(atlas_pb::ListContractsRequest {
            namespace_prefix: prefix.unwrap_or("").to_string(),
        })
        .await
        .context("ListContracts RPC")?
        .into_inner();
    if json {
        let arr: Vec<Value> = resp
            .contracts
            .iter()
            .map(|c| {
                serde_json::json!({
                    "id": c.id,
                    "version": c.version,
                    "kind": c.kind,
                    "mode": c.mode,
                    "io_msg_type": c.io_msg_type,
                    "io_srv_type": c.io_srv_type,
                    "cross_namespace": c.cross_namespace,
                    "source_toml_path": c.source_toml_path,
                    "msg_fields": c.msg_fields.iter().map(|f| serde_json::json!({
                        "name": f.name, "type_name": f.type_name,
                        "is_primitive": f.is_primitive, "is_array": f.is_array,
                        "array_size": f.array_size,
                    })).collect::<Vec<_>>(),
                    "srv_request_fields": c.srv_request_fields.iter().map(|f| serde_json::json!({
                        "name": f.name, "type_name": f.type_name,
                        "is_primitive": f.is_primitive, "is_array": f.is_array,
                        "array_size": f.array_size,
                    })).collect::<Vec<_>>(),
                    "srv_response_fields": c.srv_response_fields.iter().map(|f| serde_json::json!({
                        "name": f.name, "type_name": f.type_name,
                        "is_primitive": f.is_primitive, "is_array": f.is_array,
                        "array_size": f.array_size,
                    })).collect::<Vec<_>>(),
                })
            })
            .collect();
        println!("{}", serde_json::to_string_pretty(&arr)?);
        return Ok(());
    }
    if resp.contracts.is_empty() {
        let label = match prefix {
            Some(p) if !p.is_empty() => format!(" with prefix '{p}'"),
            _ => String::new(),
        };
        println!(
            "{} no contracts loaded{label}",
            "[contracts]".yellow().bold()
        );
        return Ok(());
    }
    for c in &resp.contracts {
        let io = if !c.io_msg_type.is_empty() {
            c.io_msg_type.clone()
        } else if !c.io_srv_type.is_empty() {
            c.io_srv_type.clone()
        } else {
            "(none)".dimmed().to_string()
        };
        println!(
            "● {}  {} {} {}{}",
            c.id.bold(),
            format!("[{}]", c.kind).dimmed(),
            format!("mode={}", c.mode).cyan(),
            format!("idl={io}").dimmed(),
            if c.cross_namespace {
                " cross-namespace".dimmed().to_string()
            } else {
                String::new()
            },
        );
        if verbose {
            if !c.msg_fields.is_empty() {
                for f in &c.msg_fields {
                    let arr = if f.is_array {
                        if f.array_size == 0 {
                            "[]".to_string()
                        } else {
                            format!("[{}]", f.array_size)
                        }
                    } else {
                        String::new()
                    };
                    println!("    {} : {}{arr}", f.name, f.type_name);
                }
            }
            if !c.srv_request_fields.is_empty() || !c.srv_response_fields.is_empty() {
                println!("    {}", "request:".dimmed());
                for f in &c.srv_request_fields {
                    println!("      {} : {}", f.name, f.type_name);
                }
                println!("    {}", "response:".dimmed());
                for f in &c.srv_response_fields {
                    println!("      {} : {}", f.name, f.type_name);
                }
            }
            if !c.source_toml_path.is_empty() {
                println!("    {} {}", "src:".dimmed(), c.source_toml_path.dimmed());
            }
        }
    }
    if !verbose {
        println!(
            "\n{} {} contract(s); pass -v for field schemas + source paths",
            "[contracts]".green().bold(),
            resp.contracts.len()
        );
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
