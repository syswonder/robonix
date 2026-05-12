// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Contract registry — loads `<robonix_source>/capabilities/**/*.toml` at
// atlas startup and serves their parsed metadata to clients via
// QueryContract / ListContracts. Clients no longer walk the filesystem
// or parse contract TOMLs themselves; atlas is the single source of
// truth for "what contracts exist and what's their wire shape".
//
// Scope is deliberately small: only the fields current TOMLs actually
// carry (`[contract]` id/version/kind, `[mode]` type, `[io.msg].msg`,
// `[io.srv].srv`). Richer metadata (summary / examples / safety /
// capability-card-style fields) waits until the TOML schema grows.

use anyhow::Context;
use log::{info, warn};
use serde::Deserialize;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use walkdir::WalkDir;

use robonix_codegen::codegen::msg_parser::{
    MsgResolver, MsgSpec, MsgTypeRef, ResolveContext, parse_ridl_type_ref,
};

use crate::pb;

#[derive(Debug, Deserialize)]
struct RawContract {
    contract: ContractSection,
    #[serde(default)]
    mode: Option<ModeSection>,
    #[serde(default)]
    io: Option<IoSection>,
}

#[derive(Debug, Deserialize)]
struct ContractSection {
    id: String,
    #[serde(default)]
    version: Option<String>,
    #[serde(default)]
    kind: Option<String>,
    /// Lib-relative IDL path (with extension), e.g. `sensor_msgs/msg/Image.msg`
    /// or `pilot/srv/SubmitTask.srv`. Source of truth in the post-migration
    /// schema; `[io.msg]` / `[io.srv]` are legacy and only honoured when
    /// `idl` is absent so old TOMLs keep loading until they're rewritten.
    #[serde(default)]
    idl: Option<String>,
    /// Generic one-line natural-language description for this contract
    /// (what it does in the abstract). Consumers MERGE this with each
    /// CapabilityProvider's instance-specific
    /// `DeclareCapabilityRequest.description` at consume time -- the
    /// two are complementary (generic + instance-specific), not
    /// alternatives.
    #[serde(default)]
    description: Option<String>,
}

#[derive(Debug, Deserialize)]
struct ModeSection {
    #[serde(default, rename = "type")]
    ty: Option<String>,
}

#[derive(Debug, Deserialize)]
struct IoSection {
    #[serde(default)]
    msg: Option<MsgSubsection>,
    #[serde(default)]
    srv: Option<SrvSubsection>,
}

#[derive(Debug, Deserialize)]
struct MsgSubsection {
    #[serde(default)]
    msg: Option<String>,
}

#[derive(Debug, Deserialize)]
struct SrvSubsection {
    #[serde(default)]
    srv: Option<String>,
}

/// In-memory contract metadata. Built once at startup; never mutated
/// while atlas runs. Wrapped in `Arc<ContractRegistry>` so handlers can
/// read it without locks.
#[derive(Debug, Default)]
pub struct ContractRegistry {
    by_id: HashMap<String, pb::ContractDescriptor>,
}

impl ContractRegistry {
    /// Walk `<root>/**/*.toml` for one root and load it into the registry.
    /// Convenience wrapper around `load_from_capability_roots(&[root])`.
    pub fn load_from_capabilities_dir(root: &Path) -> anyhow::Result<Self> {
        Self::load_from_capability_roots(std::slice::from_ref(&root))
    }

    /// Walk every `<root>/**/*.toml` across all `roots` (in order) and
    /// merge the parsed `ContractDescriptor`s into one registry. Later
    /// roots override earlier ones on duplicate `[contract].id`, which
    /// matches the package-merge semantics: per-package
    /// `<pkg>/capabilities/` can re-declare a contract from the global
    /// `<robonix_source>/capabilities/`. Symlinks are followed so msg/srv
    /// directories that live in `interfaces/lib/...` are reachable when
    /// they are linked under `capabilities/`.
    ///
    /// After all TOMLs are loaded, this also indexes every `.msg`/`.srv`
    /// under `<root>/lib/**/*` and tries to attach top-level field
    /// schemas to each contract whose io_msg_type / io_srv_type points
    /// at a known IDL. Failures here are non-fatal (the contract is
    /// still served, just without field-level introspection).
    ///
    /// Malformed TOMLs and `*.toml` files without a `[contract].id` are
    /// logged and skipped — one bad file must not take atlas down.
    pub fn load_from_capability_roots(roots: &[&Path]) -> anyhow::Result<Self> {
        let mut by_id: HashMap<String, pb::ContractDescriptor> = HashMap::new();
        let mut total_roots_walked = 0usize;
        for root in roots {
            if !root.exists() {
                warn!(
                    "[atlas] contract registry: capabilities root missing: {} \
                     (skipping)",
                    root.display()
                );
                continue;
            }
            total_roots_walked += 1;
            let mut loaded_from_root = 0usize;
            for entry in WalkDir::new(root)
                .follow_links(true)
                .into_iter()
                .filter_entry(|e| {
                    // Hard convention: `<capabilities>/lib/` holds only
                    // ROS msg/srv source for IDL codegen. Skip it so any
                    // stray .toml under lib/ never lands in the contract
                    // registry.
                    !(e.file_type().is_dir() && e.file_name() == "lib" && e.depth() > 0)
                })
                .filter_map(|e| e.ok())
            {
                if !entry.file_type().is_file() {
                    continue;
                }
                let path = entry.path();
                if path.extension().and_then(|s| s.to_str()) != Some("toml") {
                    continue;
                }
                match load_one(path) {
                    Ok(desc) => {
                        let id = desc.id.clone();
                        if let Some(prev) = by_id.insert(id.clone(), desc) {
                            warn!(
                                "[atlas] contract registry: duplicate id '{id}' \
                                 (was {}, now {}); keeping latest",
                                prev.source_toml_path,
                                path.display()
                            );
                        }
                        loaded_from_root += 1;
                    }
                    Err(e) => warn!("[atlas] contract registry: skip {} ({e:#})", path.display()),
                }
            }
            info!(
                "[atlas] contract registry: {} contracts from {}",
                loaded_from_root,
                root.display()
            );
        }
        info!(
            "[atlas] contract registry: total {} unique contracts across {} root(s)",
            by_id.len(),
            total_roots_walked
        );

        attach_idl_fields(&mut by_id, roots);

        Ok(Self { by_id })
    }

    pub fn get(&self, contract_id: &str) -> Option<&pb::ContractDescriptor> {
        self.by_id.get(contract_id)
    }

    /// Return all contracts whose id starts with `prefix`. Empty prefix
    /// returns every contract.
    pub fn list_with_prefix(&self, prefix: &str) -> Vec<pb::ContractDescriptor> {
        let prefix = prefix.trim();
        let mut out: Vec<pb::ContractDescriptor> = self
            .by_id
            .values()
            .filter(|c| prefix.is_empty() || c.id.starts_with(prefix))
            .cloned()
            .collect();
        out.sort_by(|a, b| a.id.cmp(&b.id));
        out
    }

    pub fn len(&self) -> usize {
        self.by_id.len()
    }

    pub fn is_empty(&self) -> bool {
        self.by_id.is_empty()
    }
}

/// Derive (io_msg_type, io_srv_type) from a parsed contract toml. New
/// schema: `[contract].idl = "<pkg>/(msg|srv)/<Name>.<ext>"`. Old
/// schema (pre-migration): `[io.msg].msg = "..."` / `[io.srv].srv = "..."`.
/// `idl` wins when both are present; old form is the fallback so TOMLs
/// that haven't been rewritten still load.
fn io_types_from_parsed(parsed: &RawContract) -> (String, String) {
    if let Some(idl_raw) = parsed.contract.idl.as_deref() {
        let idl = idl_raw.trim();
        if !idl.is_empty()
            && let Some((pkg, kind, name)) = parse_idl_path(idl)
        {
            let composed = format!("{pkg}/{kind}/{name}");
            return match kind {
                "msg" => (composed, String::new()),
                "srv" => (String::new(), composed),
                _ => (String::new(), String::new()),
            };
        }
    }
    match &parsed.io {
        Some(io) => {
            let msg = io
                .msg
                .as_ref()
                .and_then(|m| m.msg.as_deref())
                .map(|s| s.trim().to_string())
                .unwrap_or_default();
            let srv = io
                .srv
                .as_ref()
                .and_then(|s| s.srv.as_deref())
                .map(|s| s.trim().to_string())
                .unwrap_or_default();
            (msg, srv)
        }
        None => (String::new(), String::new()),
    }
}

/// Parse a lib-relative IDL path like `sensor_msgs/msg/Image.msg`
/// into (pkg, kind, name). Mirrors the codegen-side parser
/// (`robonix-codegen::contract_gen::parse_idl_path`) but lives here so
/// atlas doesn't need a build-dep on the full codegen crate just for one
/// six-line helper.
fn parse_idl_path(s: &str) -> Option<(&str, &'static str, &str)> {
    let (stem, kind): (&str, &'static str) = if let Some(rest) = s.strip_suffix(".srv") {
        (rest, "srv")
    } else if let Some(rest) = s.strip_suffix(".msg") {
        (rest, "msg")
    } else {
        return None;
    };
    let parts: Vec<&str> = stem.split('/').filter(|p| !p.is_empty()).collect();
    if parts.is_empty() {
        return None;
    }
    let n = parts.len();
    let name = parts[n - 1];
    let pkg = if n >= 3 && (parts[n - 2] == "srv" || parts[n - 2] == "msg") {
        parts[n - 3]
    } else {
        ""
    };
    Some((pkg, kind, name))
}

fn load_one(path: &Path) -> anyhow::Result<pb::ContractDescriptor> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("read contract toml: {}", path.display()))?;
    let parsed: RawContract =
        toml::from_str(&raw).with_context(|| format!("parse contract toml: {}", path.display()))?;
    let id = parsed.contract.id.trim().to_string();
    if id.is_empty() {
        anyhow::bail!("[contract].id is empty");
    }
    let (io_msg_type, io_srv_type) = io_types_from_parsed(&parsed);
    let version = parsed
        .contract
        .version
        .map(|s| s.trim().to_string())
        .unwrap_or_default();
    let kind_str = parsed
        .contract
        .kind
        .map(|s| s.trim().to_string())
        .unwrap_or_default();
    let kind = match kind_str.as_str() {
        "primitive" => pb::Kind::Primitive,
        "service" => pb::Kind::Service,
        "skill" => pb::Kind::Skill,
        "" => pb::Kind::Unspecified,
        other => {
            return Err(anyhow::anyhow!(
                "contract '{id}': unknown kind '{other}' (want primitive|service|skill)"
            ));
        }
    };
    let mode = parsed
        .mode
        .and_then(|m| m.ty)
        .map(|s| s.trim().to_string())
        .unwrap_or_default();
    let description = parsed
        .contract
        .description
        .map(|s| s.trim().to_string())
        .unwrap_or_default();
    Ok(pb::ContractDescriptor {
        id,
        version,
        kind: kind as i32,
        mode,
        io_msg_type,
        io_srv_type,
        source_toml_path: path.to_string_lossy().into_owned(),
        description,
        // Filled later by attach_idl_fields() after every TOML has
        // been loaded. Empty here is the right default.
        msg_fields: Vec::new(),
        srv_request_fields: Vec::new(),
        srv_response_fields: Vec::new(),
    })
}

/// After all TOMLs are loaded, walk every `<root>/lib/**/*.{msg,srv}`
/// and attach top-level field schemas to each contract whose
/// `io_msg_type` / `io_srv_type` resolves to a known IDL.
///
/// Failures are logged-and-skipped: a contract with no resolvable IDL
/// (e.g. type "X" not present in any `lib/`) just keeps its empty
/// `msg_fields` / `srv_*_fields`. Atlas startup must not fail because
/// of a single missing `.msg`.
fn attach_idl_fields(by_id: &mut HashMap<String, pb::ContractDescriptor>, roots: &[&Path]) {
    // The msg_parser indexes from `include_paths`. For each capability
    // root, the IDL files live under `<root>/lib`; everything else
    // under the root is either contract TOMLs or non-IDL data.
    let lib_paths: Vec<PathBuf> = roots
        .iter()
        .map(|r| r.join("lib"))
        .filter(|p| p.exists())
        .collect();
    if lib_paths.is_empty() {
        info!("[atlas] contract registry: no <root>/lib/ found — skipping IDL field attachment");
        return;
    }
    let mut resolver = match MsgResolver::new(&lib_paths) {
        Ok(r) => r,
        Err(e) => {
            warn!(
                "[atlas] contract registry: MsgResolver init failed ({e:#}); \
                 contracts will have no field-level schema"
            );
            return;
        }
    };
    let mut msg_filled = 0usize;
    let mut srv_filled = 0usize;
    let mut msg_missing = 0usize;
    let mut srv_missing = 0usize;
    for desc in by_id.values_mut() {
        if !desc.io_msg_type.is_empty() {
            match resolve_msg_fields(&mut resolver, &desc.io_msg_type) {
                Ok(fields) => {
                    desc.msg_fields = fields;
                    msg_filled += 1;
                }
                Err(e) => {
                    warn!(
                        "[atlas] contract registry: IDL resolve failed for \
                         contract '{}' io_msg_type='{}': {e:#}",
                        desc.id, desc.io_msg_type
                    );
                    msg_missing += 1;
                }
            }
        }
        if !desc.io_srv_type.is_empty() {
            match resolve_srv_fields(&mut resolver, &desc.io_srv_type) {
                Ok((req, resp)) => {
                    desc.srv_request_fields = req;
                    desc.srv_response_fields = resp;
                    srv_filled += 1;
                }
                Err(e) => {
                    warn!(
                        "[atlas] contract registry: IDL resolve failed for \
                         contract '{}' io_srv_type='{}': {e:#}",
                        desc.id, desc.io_srv_type
                    );
                    srv_missing += 1;
                }
            }
        }
    }
    info!(
        "[atlas] contract registry: IDL fields — msg {msg_filled} ok / {msg_missing} missing, \
         srv {srv_filled} ok / {srv_missing} missing"
    );
}

/// Look up the .msg file for "pkg/msg/Name" (ROS-style fully-qualified
/// type ref) and convert its top-level fields into the wire schema.
fn resolve_msg_fields(
    resolver: &mut MsgResolver,
    type_ref: &str,
) -> anyhow::Result<Vec<pb::FieldSpec>> {
    let (pkg, name) = parse_ridl_type_ref(type_ref)
        .with_context(|| format!("not a fully-qualified IDL type ref: {type_ref}"))?;
    let ctx = ResolveContext {
        namespace: None,
        interface_kind: Some("msg"),
        interface_name: Some(name.clone()),
        field_name: None,
    };
    resolver.resolve_named_type(&pkg, &name, Some((type_ref, &ctx)))?;
    let spec = resolver
        .cache
        .get(&(pkg.clone(), name.clone()))
        .with_context(|| format!("MsgResolver cache miss for {pkg}/{name}"))?;
    Ok(spec_to_field_specs(spec))
}

/// Same for "pkg/srv/Name" → (request_fields, response_fields).
/// `parse_ridl_type_ref` accepts both `pkg/msg/Name` and
/// `pkg/srv/Name`; we don't need a separate parser anymore.
fn resolve_srv_fields(
    resolver: &mut MsgResolver,
    type_ref: &str,
) -> anyhow::Result<(Vec<pb::FieldSpec>, Vec<pb::FieldSpec>)> {
    let (pkg, name) = parse_ridl_type_ref(type_ref)
        .with_context(|| format!("not a fully-qualified srv type ref: {type_ref}"))?;
    let key = (pkg.clone(), name.clone());
    if !resolver.srv_cache.contains_key(&key) {
        let path = resolver
            .srv_index
            .get(&key)
            .cloned()
            .with_context(|| format!("MsgResolver srv_index has no entry for {pkg}/{name}"))?;
        let parsed = robonix_codegen::codegen::msg_parser::parse_srv_file(&pkg, &name, &path)?;
        resolver.srv_cache.insert(key.clone(), parsed);
    }
    let spec = resolver
        .srv_cache
        .get(&key)
        .with_context(|| format!("srv_cache miss for {pkg}/{name}"))?;
    Ok((
        spec_to_field_specs(&spec.request),
        spec_to_field_specs(&spec.response),
    ))
}

fn spec_to_field_specs(spec: &MsgSpec) -> Vec<pb::FieldSpec> {
    spec.fields
        .iter()
        .map(|f| {
            let (type_name, is_primitive) = match &f.type_ref {
                MsgTypeRef::Primitive(s) => (s.clone(), true),
                MsgTypeRef::Named { package, name } => (format!("{package}/{name}"), false),
            };
            pb::FieldSpec {
                name: f.name.clone(),
                type_name,
                is_primitive,
                is_array: f.is_array,
                array_size: f.array_size.unwrap_or(0) as u32,
            }
        })
        .collect()
}

/// Resolve the list of capability roots atlas should load. Priority:
///   1. explicit CLI/env paths (any non-empty entries from
///      `--capabilities a,b,c` or `ROBONIX_ATLAS_CAPABILITIES=a,b,c`)
///   2. `$ROBONIX_SOURCE_PATH/capabilities` as a single fallback root
///
/// Returns an empty vec if nothing is configured; atlas then runs with
/// an empty registry (handlers return found=false on every query).
///
/// Per-package `<pkg>/capabilities/` dirs aren't included here — those
/// can be added at deploy time by the rbnx CLI walking installed
/// package paths and passing the merged list via `--capabilities`.
pub fn resolve_capabilities_roots(explicit: &[String]) -> Vec<PathBuf> {
    let cleaned: Vec<PathBuf> = explicit
        .iter()
        .map(|s| s.trim())
        .filter(|s| !s.is_empty())
        .map(PathBuf::from)
        .collect();
    if !cleaned.is_empty() {
        return cleaned;
    }
    if let Ok(root) = std::env::var("ROBONIX_SOURCE_PATH") {
        let trimmed = root.trim();
        if !trimmed.is_empty() {
            return vec![PathBuf::from(trimmed).join("capabilities")];
        }
    }
    Vec::new()
}
