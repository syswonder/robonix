// SPDX-License-Identifier: MulanPSL-2.0
// Generate `robonix_contracts.proto` from `<root>/capabilities/**/*.toml`.
//
// `[mode].type` → `robonix_contracts.proto` (see `<root>/capabilities/README.md`).
// Streaming: `rpc_server_stream` uses the .srv response (exactly one field) as stream element; `rpc_client_stream` uses the request (exactly one field).

use anyhow::{Context, Result, bail};
use serde::Deserialize;
use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

use super::msg_parser::{MsgField, MsgResolver, MsgTypeRef, SrvSpec};
use super::proto_gen::proto_package_name;

#[derive(Debug, Deserialize)]
struct ContractToml {
    contract: ContractMeta,
    mode: ModeSpec,
    #[serde(default)]
    extra_rpc: Vec<ExtraRpc>,
}

#[derive(Debug, Deserialize, Clone)]
struct ExtraRpc {
    /// IDL reference: lib-relative path without extension. Same form as
    /// `[contract].idl` — codegen finds `<lib-root>/<idl>.{srv,msg}` in
    /// the merged lib roots.
    idl: String,
    #[serde(rename = "type")]
    mode_type: String,
    name: String,
}

#[derive(Debug, Deserialize)]
struct ContractMeta {
    id: String,
    #[allow(dead_code)]
    version: String,
    #[allow(dead_code)]
    kind: String,
    /// IDL reference: full path under one of the merged lib roots
    /// (`<robonix>/capabilities/lib/` or `<pkg>/capabilities/lib/`),
    /// without the `.srv` / `.msg` extension. The second-to-last path
    /// segment must be `srv` or `msg`. Examples:
    ///   `system/pilot/srv/SubmitTask`           → lib/.../srv/SubmitTask.srv
    ///   `common_interfaces/sensor_msgs/msg/Image` → lib/.../msg/Image.msg
    idl: String,
    /// For `rpc_bidirectional_stream` mode, the client→server stream
    /// element type (lib-relative path to a `.msg`). Required for bidi.
    #[serde(default)]
    stream_request: Option<String>,
    /// For bidi (and historically server-stream) modes, the server→client
    /// stream element type (lib-relative path to a `.msg`).
    #[serde(default)]
    stream_response: Option<String>,
}

#[derive(Debug, Deserialize)]
struct ModeSpec {
    #[serde(rename = "type")]
    mode_type: String,
}

/// Internal IDL-reference triple after parsing `<lib-relative path>` and
/// optional bidi-stream element types. Constructed in resolve_contract_io
/// from the new `[contract].idl` schema; passed to the per-mode resolvers.
struct IdlRef<'a> {
    /// Original path string from the toml field, kept for error messages.
    path: &'a str,
    /// Bidi `stream_request` field, if any.
    stream_request: Option<&'a str>,
    /// Bidi / historical `stream_response` field, if any.
    stream_response: Option<&'a str>,
}

/// `(ROS package, srv interface name)` pairs from every contract's
/// `[contract].idl` (and per-extra_rpc `idl`) when it points at a `.srv`.
/// Used by proto generation: only these `.srv` files get
/// `*_Request` / `*_Response` messages; per-package `service` RPCs are not emitted.
pub fn collect_referenced_srvs(contracts_dir: &Path) -> Result<BTreeSet<(String, String)>> {
    let paths = collect_tomls(contracts_dir)?;
    let mut set = BTreeSet::new();
    for p in paths {
        let raw =
            fs::read_to_string(&p).with_context(|| format!("read contract {}", p.display()))?;
        let c: ContractToml =
            toml::from_str(&raw).with_context(|| format!("parse TOML {}", p.display()))?;
        let idl = c.contract.idl.trim();
        match parse_idl_path(idl) {
            Some((pkg, "srv", name)) => {
                set.insert((pkg.to_string(), name.to_string()));
            }
            Some((_, "msg", _)) => {
                // `idl` points at a `.msg` (topic mode); not a srv reference.
            }
            _ => bail!(
                "contract {}: [contract].idl must be a lib-relative file path ending in .srv or .msg, got {idl:?}",
                c.contract.id
            ),
        }
        for extra in &c.extra_rpc {
            let p = extra.idl.trim();
            match parse_idl_path(p) {
                Some((pkg, "srv", name)) => {
                    set.insert((pkg.to_string(), name.to_string()));
                }
                _ => bail!(
                    "contract {}: [[extra_rpc]].idl must point to a .srv file, got {p:?}",
                    c.contract.id
                ),
            }
        }
    }
    Ok(set)
}

pub fn generate(
    resolver: &mut MsgResolver,
    contracts_dirs: &[PathBuf],
    out_dir: &Path,
    verbose: bool,
) -> Result<()> {
    let mut paths: Vec<PathBuf> = Vec::new();
    for d in contracts_dirs {
        for p in collect_tomls(d)? {
            paths.push(p);
        }
    }
    if paths.is_empty() {
        if verbose {
            for d in contracts_dirs {
                eprintln!(
                    "[robonix-codegen] contracts: no .toml under {}",
                    d.display()
                );
            }
        }
        return Ok(());
    }

    // De-dup on contract id: later root wins, matching atlas's
    // contract-registry merge semantics. This lets a per-package
    // contract override a global one of the same id during codegen.
    let mut by_id: std::collections::BTreeMap<String, (PathBuf, ContractToml)> =
        std::collections::BTreeMap::new();
    for p in paths {
        let raw =
            fs::read_to_string(&p).with_context(|| format!("read contract {}", p.display()))?;
        let c: ContractToml =
            toml::from_str(&raw).with_context(|| format!("parse TOML {}", p.display()))?;
        by_id.insert(c.contract.id.clone(), (p, c));
    }
    let mut contracts: Vec<(PathBuf, ContractToml)> = by_id.into_values().collect();
    contracts.sort_by(|a, b| a.1.contract.id.cmp(&b.1.contract.id));

    let mut out = String::new();
    writeln!(&mut out, "// @generated by robonix-codegen (--contracts).")?;
    writeln!(&mut out, "// Do not edit by hand.")?;
    writeln!(&mut out, "syntax = \"proto3\";")?;
    writeln!(&mut out)?;
    writeln!(&mut out, "package robonix.contracts;")?;
    writeln!(&mut out)?;
    writeln!(&mut out, "import \"google/protobuf/empty.proto\";")?;
    writeln!(&mut out)?;

    let mut imports: BTreeSet<String> = BTreeSet::new();
    let mut needs_string_wire = false;

    let mut proto_types: Vec<(String, ResolvedType, ResolvedType)> = Vec::new();
    let mut extra_rpcs_resolved: Vec<Vec<(String, String, ResolvedType, ResolvedType)>> =
        Vec::new();
    for (_, c) in &contracts {
        let (in_t, out_t) = resolve_contract_io(c, resolver, &mut imports, &mut needs_string_wire)?;
        proto_types.push((c.contract.id.clone(), in_t, out_t));

        let mut extras = Vec::new();
        for extra in &c.extra_rpc {
            let (ein, eout) = resolve_extra_rpc(
                extra,
                &c.contract.id,
                resolver,
                &mut imports,
                &mut needs_string_wire,
            )?;
            extras.push((extra.name.clone(), extra.mode_type.clone(), ein, eout));
        }
        extra_rpcs_resolved.push(extras);
    }

    for imp in &imports {
        writeln!(&mut out, "import \"{imp}\";",)?;
    }
    if !imports.is_empty() {
        writeln!(&mut out)?;
    }

    if needs_string_wire {
        writeln!(
            &mut out,
            "// Wrapper for contracts that use primitive/string until shared IDL exists."
        )?;
        writeln!(&mut out, "message StringWire {{")?;
        writeln!(&mut out, "  string value = 1;")?;
        writeln!(&mut out, "}}")?;
        writeln!(&mut out)?;
    }

    for (idx, ((_, c), (_, in_t, out_t))) in contracts.iter().zip(proto_types.iter()).enumerate() {
        let mode = c.mode.mode_type.trim();
        let svc = contract_id_to_service_name(&c.contract.id);
        // RPC method name:
        //   - srv-backed contracts (rpc / rpc_*_stream): use the .srv
        //     filename basename (canonical PascalCase). Existing
        //     consumers across the codebase expect this.
        //   - msg-backed contracts (topic_in / topic_out): use the
        //     contract_id leaf (e.g. `robonix/primitive/audio/mic` →
        //     `mic` → CamelCased to `Mic`). Don't use the .msg basename
        //     (`AudioChunk` etc.) — that would change the wire-level
        //     gRPC method name and break existing client code.
        let idl_kind = parse_idl_path(c.contract.idl.trim()).map(|(_, kind, _)| kind);
        let method_raw = if idl_kind == Some("srv") {
            parse_idl_path(c.contract.idl.trim())
                .map(|(_, _, name)| name.to_string())
                .unwrap_or_else(|| c.contract.id.clone())
        } else {
            c.contract
                .id
                .rsplit_once('/')
                .map(|(_, leaf)| leaf.to_string())
                .unwrap_or_else(|| c.contract.id.clone())
        };
        // RPC method names must be UpperCamelCase. `.srv` filenames are
        // already CamelCase by ROS convention so this is identity for
        // them; the fallback (contract id leaf, e.g. `scan_2d` for
        // topic-style contracts) gets normalised here.
        let method = upper_camel(&method_raw);
        writeln!(
            &mut out,
            "// contract: {} (v{})",
            c.contract.id, c.contract.version
        )?;
        writeln!(&mut out, "service {svc} {{")?;

        let rpc = match mode {
            "rpc" => format_unary(&method, in_t, out_t),
            "rpc_server_stream" | "topic_out" => format_stream_out(&method, in_t, out_t),
            "rpc_client_stream" | "topic_in" => format_stream_in(&method, in_t, out_t),
            "rpc_bidirectional_stream" => format_bidi_stream(&method, in_t, out_t),
            other => bail!(
                "unknown [mode].type '{other}' in contract {} (expected rpc | rpc_server_stream | rpc_client_stream | topic_out | topic_in)",
                c.contract.id
            ),
        };
        writeln!(&mut out, "  {rpc}")?;

        for (name, extra_mode, ein, eout) in &extra_rpcs_resolved[idx] {
            let extra_rpc = format_named_rpc(name, extra_mode, ein, eout);
            writeln!(&mut out, "  {extra_rpc}")?;
        }

        writeln!(&mut out, "}}")?;
        writeln!(&mut out)?;
    }

    let outfile = out_dir.join("robonix_contracts.proto");
    fs::write(&outfile, &out).with_context(|| format!("write {}", outfile.display()))?;
    if verbose {
        eprintln!(
            "[robonix-codegen] contracts: wrote {} ({} services)",
            outfile.display(),
            contracts.len()
        );
    }

    super::contract_proto_modules_gen::write(out_dir, verbose)?;
    Ok(())
}

#[derive(Clone)]
enum ResolvedType {
    ProtoFqn(String),
    GoogleEmpty,
    /// Reserved escape hatch for raw string-typed contracts. Currently
    /// unused (no contract opts in); kept so the plumbing is in place
    /// for the rare case it's needed.
    #[allow(dead_code)]
    StringWire,
}

fn resolve_extra_rpc(
    extra: &ExtraRpc,
    contract_id: &str,
    resolver: &mut MsgResolver,
    imports: &mut BTreeSet<String>,
    needs_string_wire: &mut bool,
) -> Result<(ResolvedType, ResolvedType)> {
    let idl = IdlRef {
        path: extra.idl.trim(),
        stream_request: None,
        stream_response: None,
    };
    match extra.mode_type.trim() {
        "rpc" => resolve_srv_contract_pair(idl.path, resolver, imports, needs_string_wire),
        "rpc_server_stream" => {
            resolve_srv_server_stream(&idl, contract_id, resolver, imports, needs_string_wire)
        }
        "rpc_client_stream" => {
            resolve_srv_client_stream(&idl, contract_id, resolver, imports, needs_string_wire)
        }
        other => bail!("contract {contract_id}: [[extra_rpc]] unknown type '{other}'"),
    }
}

fn format_named_rpc(name: &str, mode: &str, input: &ResolvedType, output: &ResolvedType) -> String {
    match mode.trim() {
        "rpc" => format!(
            "rpc {name}({}) returns ({});",
            unary_arg(input),
            unary_return(output)
        ),
        "rpc_server_stream" => format!(
            "rpc {name}({}) returns (stream {});",
            empty_or_type(input),
            stream_element(output)
        ),
        "rpc_client_stream" => format!(
            "rpc {name}(stream {}) returns ({});",
            stream_element(input),
            unary_return(output)
        ),
        "rpc_bidirectional_stream" => format!(
            "rpc {name}(stream {}) returns (stream {});",
            stream_element(input),
            stream_element(output)
        ),
        _ => format!("// unknown mode for extra_rpc {name}"),
    }
}

fn format_stream_out(method: &str, input: &ResolvedType, output: &ResolvedType) -> String {
    format!(
        "rpc {method}({}) returns (stream {});",
        empty_or_type(input),
        stream_element(output)
    )
}

fn format_stream_in(method: &str, input: &ResolvedType, output: &ResolvedType) -> String {
    format!(
        "rpc {method}(stream {}) returns ({});",
        stream_element(input),
        unary_return(output)
    )
}

fn format_bidi_stream(method: &str, input: &ResolvedType, output: &ResolvedType) -> String {
    format!(
        "rpc {method}(stream {}) returns (stream {});",
        stream_element(input),
        stream_element(output)
    )
}

fn format_unary(method: &str, input: &ResolvedType, output: &ResolvedType) -> String {
    format!(
        "rpc {method}({}) returns ({});",
        unary_arg(input),
        unary_return(output)
    )
}

fn empty_or_type(t: &ResolvedType) -> String {
    match t {
        ResolvedType::GoogleEmpty => "google.protobuf.Empty".to_string(),
        ResolvedType::ProtoFqn(s) => s.clone(),
        ResolvedType::StringWire => "robonix.contracts.StringWire".to_string(),
    }
}

fn unary_arg(t: &ResolvedType) -> String {
    empty_or_type(t)
}

fn unary_return(t: &ResolvedType) -> String {
    match t {
        ResolvedType::GoogleEmpty => "google.protobuf.Empty".to_string(),
        ResolvedType::ProtoFqn(s) => s.clone(),
        ResolvedType::StringWire => "robonix.contracts.StringWire".to_string(),
    }
}

fn stream_element(t: &ResolvedType) -> String {
    unary_arg(t)
}

fn srv_stream_field_to_resolved(
    contract_id: &str,
    srv_path: &str,
    section: &str,
    field: &MsgField,
    resolver: &mut MsgResolver,
    imports: &mut BTreeSet<String>,
    needs_string_wire: &mut bool,
) -> Result<ResolvedType> {
    if field.is_array {
        bail!(
            "contract {contract_id}: [{section}] stream element must be a single message, not an array (in {srv_path})"
        );
    }
    field_to_resolved_type(field, resolver, imports, needs_string_wire)
}

fn resolve_contract_io(
    c: &ContractToml,
    resolver: &mut MsgResolver,
    imports: &mut BTreeSet<String>,
    needs_string_wire: &mut bool,
) -> Result<(ResolvedType, ResolvedType)> {
    let mode = c.mode.mode_type.trim();
    let idl_path = c.contract.idl.trim();
    let (_, kind, _) = parse_idl_path(idl_path).ok_or_else(|| {
        anyhow::anyhow!(
            "contract {}: [contract].idl must be a lib-relative file path ending in .srv or .msg, got {idl_path:?}",
            c.contract.id
        )
    })?;

    // Verify the file actually exists at this path under at least one
    // configured lib root. The `idl` field is interpreted as the literal
    // lib-relative file path (with extension); codegen joins each
    // lib_root with this path and checks file existence.
    if !idl_path_exists(idl_path, resolver) {
        bail!(
            "contract {}: idl path {idl_path:?} doesn't resolve to a file under any lib root ({})",
            c.contract.id,
            resolver.include_paths.len()
        );
    }

    let idl = IdlRef {
        path: idl_path,
        stream_request: c.contract.stream_request.as_deref(),
        stream_response: c.contract.stream_response.as_deref(),
    };

    match (mode, kind) {
        ("rpc", "srv") => resolve_srv_contract_pair(idl.path, resolver, imports, needs_string_wire),
        ("rpc_server_stream", "srv") => {
            resolve_srv_server_stream(&idl, &c.contract.id, resolver, imports, needs_string_wire)
        }
        ("rpc_client_stream", "srv") => {
            resolve_srv_client_stream(&idl, &c.contract.id, resolver, imports, needs_string_wire)
        }
        ("rpc_bidirectional_stream", "srv") => {
            resolve_srv_bidi_stream(&idl, &c.contract.id, resolver, imports, needs_string_wire)
        }
        ("topic_out", "msg") => {
            let elem = resolve_io(idl.path, resolver, imports, needs_string_wire)?;
            Ok((ResolvedType::GoogleEmpty, elem))
        }
        ("topic_in", "msg") => {
            let elem = resolve_io(idl.path, resolver, imports, needs_string_wire)?;
            Ok((elem, ResolvedType::GoogleEmpty))
        }
        ("rpc" | "rpc_server_stream" | "rpc_client_stream" | "rpc_bidirectional_stream", "msg") => {
            bail!(
                "contract {}: mode={mode:?} requires a `.srv` IDL but [contract].idl points at a `.msg` ({idl_path:?})",
                c.contract.id
            )
        }
        ("topic_out" | "topic_in", "srv") => {
            bail!(
                "contract {}: mode={mode:?} requires a `.msg` IDL but [contract].idl points at a `.srv` ({idl_path:?})",
                c.contract.id
            )
        }
        (other, _) => bail!(
            "unknown [mode].type {other:?} in contract {}",
            c.contract.id
        ),
    }
}

/// Parse the user-written `idl` path. The path includes the file extension
/// (`.srv` / `.msg`) and is interpreted as the literal lib-relative file
/// path — codegen will look it up at `<lib_root>/<idl>` for each lib root.
///
/// Returns `(pkg, kind, name)` where:
///   - `kind` is derived from the file extension (`"srv"` / `"msg"`)
///   - `name` is the file basename without extension (e.g. `SubmitTask`)
///   - `pkg` is the directory immediately above `srv/` / `msg/` when
///     the path follows the conventional `<...>/<pkg>/{srv,msg}/<Name>`
///     layout — needed by the downstream MsgResolver lookup. For flat
///     layouts (no `/srv/` or `/msg/` subdir), `pkg` is empty: the
///     existing resolver doesn't index those, and the caller will get
///     a clear "not indexed" error from the resolver itself.
fn parse_idl_path(
    s: &str,
) -> Option<(
    &str,         /* pkg */
    &'static str, /* kind */
    &str,         /* name */
)> {
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

/// Verify the user-written idl path resolves to an actual file under
/// at least one of the resolver's include_paths. The path is the literal
/// file path — codegen joins it directly with each lib root.
fn idl_path_exists(idl: &str, resolver: &MsgResolver) -> bool {
    for root in &resolver.include_paths {
        if root.join(idl).is_file() {
            return true;
        }
    }
    false
}

fn resolve_srv_server_stream(
    idl: &IdlRef,
    contract_id: &str,
    resolver: &mut MsgResolver,
    imports: &mut BTreeSet<String>,
    needs_string_wire: &mut bool,
) -> Result<(ResolvedType, ResolvedType)> {
    let p = idl.path;
    let Some((pkg, "srv", name)) = parse_idl_path(p) else {
        bail!("[contract].idl must end with /srv/Name for rpc modes, got {p:?}");
    };
    resolver
        .resolve_srv(pkg, name)
        .with_context(|| format!("resolve srv {p}"))?;
    let spec = resolver
        .srv_spec(pkg, name)
        .ok_or_else(|| anyhow::anyhow!("internal: srv {p} not cached"))?
        .clone();

    let res = &spec.response;
    if res.fields.len() != 1 {
        bail!(
            "contract {contract_id}: [mode] rpc_server_stream requires the .srv response section to have exactly one field (stream element type), got {} in {p}",
            res.fields.len()
        );
    }
    let in_t = srv_request_to_contract_input(&spec, resolver, imports, needs_string_wire)?;
    let out_t = srv_stream_field_to_resolved(
        contract_id,
        p,
        "response",
        &res.fields[0],
        resolver,
        imports,
        needs_string_wire,
    )?;
    Ok((in_t, out_t))
}

fn resolve_srv_client_stream(
    idl: &IdlRef,
    contract_id: &str,
    resolver: &mut MsgResolver,
    imports: &mut BTreeSet<String>,
    needs_string_wire: &mut bool,
) -> Result<(ResolvedType, ResolvedType)> {
    let p = idl.path;
    let Some((pkg, "srv", name)) = parse_idl_path(p) else {
        bail!("[contract].idl must end with /srv/Name for rpc modes, got {p:?}");
    };
    resolver
        .resolve_srv(pkg, name)
        .with_context(|| format!("resolve srv {p}"))?;
    let spec = resolver
        .srv_spec(pkg, name)
        .ok_or_else(|| anyhow::anyhow!("internal: srv {p} not cached"))?
        .clone();

    let req = &spec.request;
    if req.fields.len() != 1 {
        bail!(
            "contract {contract_id}: [mode] rpc_client_stream requires the .srv request section to have exactly one field (stream element type), got {} in {p}",
            req.fields.len()
        );
    }
    let in_t = srv_stream_field_to_resolved(
        contract_id,
        p,
        "request",
        &req.fields[0],
        resolver,
        imports,
        needs_string_wire,
    )?;
    let out_t = srv_response_to_contract_output(&spec, resolver, imports, needs_string_wire)?;
    Ok((in_t, out_t))
}

/// Bidirectional stream: uses `stream_request` and `stream_response` from the
/// `[contract]` table (siblings of `idl`) as stream element types. Both
/// values are lib-relative paths to `.msg` files (same form as `idl`).
fn resolve_srv_bidi_stream(
    idl: &IdlRef,
    contract_id: &str,
    _resolver: &mut MsgResolver,
    imports: &mut BTreeSet<String>,
    _needs_string_wire: &mut bool,
) -> Result<(ResolvedType, ResolvedType)> {
    let in_ref = idl
        .stream_request
        .ok_or_else(|| anyhow::anyhow!(
            "contract {contract_id}: [mode] rpc_bidirectional_stream requires [contract].stream_request"
        ))?;
    let out_ref = idl
        .stream_response
        .ok_or_else(|| anyhow::anyhow!(
            "contract {contract_id}: [mode] rpc_bidirectional_stream requires [contract].stream_response"
        ))?;

    let in_t = resolve_io(in_ref, _resolver, imports, _needs_string_wire)?;
    let out_t = resolve_io(out_ref, _resolver, imports, _needs_string_wire)?;
    Ok((in_t, out_t))
}

fn srv_request_to_contract_input(
    srv: &SrvSpec,
    resolver: &mut MsgResolver,
    imports: &mut BTreeSet<String>,
    needs_string_wire: &mut bool,
) -> Result<ResolvedType> {
    let req = &srv.request;
    if req.fields.len() == 1 {
        return field_to_resolved_type(&req.fields[0], resolver, imports, needs_string_wire);
    }
    imports.insert(format!("{}.proto", srv.package));
    Ok(ResolvedType::ProtoFqn(format!(
        "{}.{}",
        proto_package_name(&srv.package),
        req.name
    )))
}

/// Empty `.srv` response section → `google.protobuf.Empty`; else the generated `*_Response` message.
fn srv_response_to_contract_output(
    srv: &SrvSpec,
    resolver: &mut MsgResolver,
    imports: &mut BTreeSet<String>,
    _needs_string_wire: &mut bool,
) -> Result<ResolvedType> {
    let res = &srv.response;
    if res.fields.is_empty() {
        return Ok(ResolvedType::GoogleEmpty);
    }
    for f in &res.fields {
        if let MsgTypeRef::Named { package, name } = &f.type_ref {
            resolver.resolve_named_type(package, name, None)?;
        }
    }
    imports.insert(format!("{}.proto", srv.package));
    Ok(ResolvedType::ProtoFqn(format!(
        "{}.{}",
        proto_package_name(&srv.package),
        res.name
    )))
}

fn resolve_srv_contract_pair(
    path: &str,
    resolver: &mut MsgResolver,
    imports: &mut BTreeSet<String>,
    _needs_string_wire: &mut bool,
) -> Result<(ResolvedType, ResolvedType)> {
    let p = path.trim();
    if let Some((pkg, "srv", name)) = parse_idl_path(p) {
        resolver
            .resolve_srv(pkg, name)
            .with_context(|| format!("resolve srv {p}"))?;
        imports.insert(format!("{pkg}.proto"));
        let req = format!("{name}_Request");
        let res = format!("{name}_Response");
        return Ok((
            ResolvedType::ProtoFqn(format!("{}.{}", proto_package_name(pkg), req)),
            ResolvedType::ProtoFqn(format!("{}.{}", proto_package_name(pkg), res)),
        ));
    }
    bail!("[contract].idl must end with /srv/Name for rpc modes, got {p:?}");
}

fn field_to_resolved_type(
    field: &MsgField,
    resolver: &mut MsgResolver,
    imports: &mut BTreeSet<String>,
    needs_string_wire: &mut bool,
) -> Result<ResolvedType> {
    match &field.type_ref {
        MsgTypeRef::Primitive(_) => bail!(
            "contract I/O field `{}` must use a named ROS message type, not a primitive",
            field.name
        ),
        MsgTypeRef::Named { package, name } => resolve_io(
            &format!("{package}/msg/{name}"),
            resolver,
            imports,
            needs_string_wire,
        ),
    }
}

/// Resolve a nested ROS-style type reference from inside a .srv/.msg
/// file (3-segment `pkg/msg/Name` or `pkg/srv/Name`). Different from
/// the user-facing top-level `idl` field, which uses the new
/// extension-bearing path format and is resolved via `parse_idl_path`.
fn resolve_io(
    spec: &str,
    resolver: &mut MsgResolver,
    imports: &mut BTreeSet<String>,
    _needs_string_wire: &mut bool,
) -> Result<ResolvedType> {
    let s = spec.trim();
    // First try the new extension-bearing path format (used when this
    // function is called from topic_out / topic_in / bidi resolvers
    // with the user's `idl` field value).
    if (s.ends_with(".srv") || s.ends_with(".msg"))
        && let Some((pkg, kind, name)) = parse_idl_path(s)
    {
        return match kind {
            "msg" => {
                resolver
                    .resolve_named_type(pkg, name, None)
                    .with_context(|| {
                        format!("resolve msg {pkg}/{name} referenced from contract")
                    })?;
                imports.insert(format!("{pkg}.proto"));
                Ok(ResolvedType::ProtoFqn(format!(
                    "{}.{}",
                    proto_package_name(pkg),
                    name
                )))
            }
            "srv" => {
                resolver.resolve_srv(pkg, name).with_context(|| {
                    format!("resolve srv {pkg}/{name} referenced from contract")
                })?;
                imports.insert(format!("{pkg}.proto"));
                let req = format!("{}_Request", name);
                Ok(ResolvedType::ProtoFqn(format!(
                    "{}.{}",
                    proto_package_name(pkg),
                    req
                )))
            }
            _ => unreachable!(),
        };
    }
    // Otherwise: nested ROS-style reference inside an IDL file
    // (`pkg/msg/Name` / `pkg/Name` for same-pkg refs handled by parser).
    let parts: Vec<&str> = s.split('/').collect();
    match parts.as_slice() {
        [pkg, "msg", name] => {
            resolver
                .resolve_named_type(pkg, name, None)
                .with_context(|| format!("resolve msg {pkg}/{name} referenced from contract"))?;
            imports.insert(format!("{pkg}.proto"));
            Ok(ResolvedType::ProtoFqn(format!(
                "{}.{}",
                proto_package_name(pkg),
                name
            )))
        }
        [pkg, "srv", name] => {
            resolver
                .resolve_srv(pkg, name)
                .with_context(|| format!("resolve srv {pkg}/{name} referenced from contract"))?;
            imports.insert(format!("{pkg}.proto"));
            let req = format!("{}_Request", name);
            Ok(ResolvedType::ProtoFqn(format!(
                "{}.{}",
                proto_package_name(pkg),
                req
            )))
        }
        _ => bail!(
            "unsupported IDL reference {s:?} (expected `<pkg>/msg/<Name>` or `<pkg>/srv/<Name>` for nested refs, or a lib-relative path ending in .srv/.msg for top-level idl)"
        ),
    }
}

#[allow(dead_code)]
fn parse_ros_path(s: &str) -> Option<(&str, &str, &str)> {
    let parts: Vec<&str> = s.split('/').collect();
    if parts.len() != 3 {
        return None;
    }
    Some((parts[0], parts[1], parts[2]))
}

/// Convert an arbitrary identifier to UpperCamelCase. Splits on `_`/`-`/
/// digit-letter boundaries and capitalises each segment.
/// `submit_task` → `SubmitTask`; `scan_2d` → `Scan2d`; `SubmitTask` → `SubmitTask`.
fn upper_camel(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut capitalize_next = true;
    for ch in s.chars() {
        if ch == '_' || ch == '-' {
            capitalize_next = true;
            continue;
        }
        if capitalize_next {
            out.extend(ch.to_uppercase());
            capitalize_next = false;
        } else {
            out.push(ch);
        }
    }
    out
}

fn contract_id_to_service_name(id: &str) -> String {
    let body = id.strip_prefix("robonix/").unwrap_or(id);
    body.split('/')
        .filter(|x| !x.is_empty())
        .map(|seg| {
            seg.split('_')
                .filter(|p| !p.is_empty())
                .map(|p| {
                    let mut c = p.chars();
                    match c.next() {
                        None => String::new(),
                        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
                    }
                })
                .collect::<String>()
        })
        .collect::<String>()
}

fn collect_tomls(dir: &Path) -> Result<Vec<PathBuf>> {
    let mut v = Vec::new();
    collect_tomls_inner(dir, &mut v)?;
    v.sort();
    Ok(v)
}

fn collect_tomls_inner(dir: &Path, out: &mut Vec<PathBuf>) -> Result<()> {
    if !dir.is_dir() {
        bail!("contracts directory does not exist: {}", dir.display());
    }
    for entry in fs::read_dir(dir).with_context(|| format!("read_dir {}", dir.display()))? {
        let entry = entry?;
        let p = entry.path();
        if p.is_dir() {
            // Hard convention: `<capabilities>/lib/` holds only ROS
            // msg/srv source for the IDL resolver. Skip it here so
            // any stray .toml dropped under lib/ never gets picked up
            // as a contract.
            if p.file_name().and_then(|s| s.to_str()) == Some("lib") {
                continue;
            }
            collect_tomls_inner(&p, out)?;
        } else if p.extension().and_then(|x| x.to_str()) == Some("toml") {
            out.push(p);
        }
    }
    Ok(())
}
