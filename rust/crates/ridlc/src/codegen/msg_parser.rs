// SPDX-License-Identifier: MulanPSL-2.0
// Shared ROS .msg / .srv file parser — used by rust_gen, proto_gen, etc.

use anyhow::{Context, Result, bail};
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

use super::err::RIDLC_ERR_PREFIX;

#[derive(Clone, Debug)]
pub struct MsgField {
    pub name: String,
    pub type_ref: MsgTypeRef,
    pub is_array: bool,
}

#[derive(Clone, Debug)]
pub enum MsgTypeRef {
    Primitive(String),
    Named { package: String, name: String },
}

#[derive(Clone, Debug)]
pub struct MsgSpec {
    pub package: String,
    pub name: String,
    pub fields: Vec<MsgField>,
}

/// gRPC mapping for pub-sub-style interfaces (ROS has no native streaming `.srv` wire format;
/// this directive is **codegen-only** for `ridlc --lang proto`).
#[derive(Clone, Debug)]
pub enum GrpcStreamMode {
    /// Same semantics as a ROS **publisher**: provider pushes a stream of `element` messages.
    /// gRPC: `rpc Method(Request) returns (stream element);`
    ServerStream { element: MsgTypeRef },
    /// Same semantics as a ROS **subscriber**: consumer pushes a stream of `element` messages.
    /// gRPC: `rpc Method(stream element) returns (Response);`
    ClientStream { element: MsgTypeRef },
}

#[derive(Clone, Debug)]
pub struct SrvSpec {
    pub package: String,
    pub name: String,
    pub request: MsgSpec,
    pub response: MsgSpec,
    pub grpc_stream: Option<GrpcStreamMode>,
}

#[derive(Clone, Debug, Default)]
pub struct ResolveContext {
    pub namespace: Option<String>,
    pub interface_kind: Option<&'static str>,
    pub interface_name: Option<String>,
    pub field_name: Option<String>,
}

pub struct MsgResolver {
    pub index: HashMap<(String, String), PathBuf>,
    pub cache: HashMap<(String, String), MsgSpec>,
    pub srv_index: HashMap<(String, String), PathBuf>,
    pub srv_cache: HashMap<(String, String), SrvSpec>,
    pub include_paths: Vec<PathBuf>,
}

impl MsgResolver {
    pub fn new(include_paths: &[PathBuf]) -> Result<Self> {
        let mut index = HashMap::new();
        let mut srv_index = HashMap::new();
        for root in include_paths {
            index_msg_files(root, &mut index)?;
            index_srv_files(root, &mut srv_index)?;
        }
        Ok(Self {
            index,
            cache: HashMap::new(),
            srv_index,
            srv_cache: HashMap::new(),
            include_paths: include_paths.to_vec(),
        })
    }

    pub fn resolve_ridl_type(&mut self, type_ref: &str, ctx: &ResolveContext) -> Result<()> {
        if let Some((package, name)) = parse_ridl_type_ref(type_ref) {
            self.resolve_named_type(&package, &name, Some((type_ref, ctx)))?;
        }
        Ok(())
    }

    pub fn resolve_named_type(
        &mut self,
        package: &str,
        name: &str,
        from: Option<(&str, &ResolveContext)>,
    ) -> Result<()> {
        let key = (package.to_string(), name.to_string());
        if self.cache.contains_key(&key) {
            return Ok(());
        }
        let full_type = ros_msg_type_fmt(package, name);
        let path = self
            .find_msg_path(package, name)
            .with_context(|| format_resolve_error(&full_type, from, &self.include_paths))?;
        let spec = parse_msg_file(package, name, &path)?;
        for field in &spec.fields {
            if let MsgTypeRef::Named { package, name } = &field.type_ref {
                let dep_ctx = ResolveContext {
                    namespace: Some(spec.package.clone()),
                    interface_kind: Some("msg"),
                    interface_name: Some(spec.name.clone()),
                    field_name: Some(field.name.clone()),
                };
                self.resolve_named_type(
                    package,
                    name,
                    Some((&ros_msg_type_fmt(package, name), &dep_ctx)),
                )?;
            }
        }
        self.cache.insert(key, spec);
        Ok(())
    }

    pub fn resolve_all_in_index(&mut self) -> Result<()> {
        let keys: Vec<_> = self.index.keys().cloned().collect();
        for (package, name) in keys {
            if let Err(e) = self.resolve_named_type(&package, &name, None) {
                eprintln!("[ridlc] warning: skipping {}/{}: {:#}", package, name, e);
            }
        }
        Ok(())
    }

    pub fn find_msg_path(&self, package: &str, name: &str) -> Option<PathBuf> {
        let candidates = package_aliases(package);
        for candidate in candidates {
            if let Some(path) = self.index.get(&(candidate.to_string(), name.to_string())) {
                return Some(path.clone());
            }
        }
        None
    }

    pub fn ordered_specs(&self) -> Vec<&MsgSpec> {
        let mut keys: Vec<_> = self.cache.keys().cloned().collect();
        keys.sort();
        keys.iter()
            .filter_map(|key| self.cache.get(key))
            .collect::<Vec<_>>()
    }

    pub fn resolve_all_srv(&mut self) -> Result<()> {
        let keys: Vec<_> = self.srv_index.keys().cloned().collect();
        for (package, name) in keys {
            if let Err(e) = self.resolve_srv(&package, &name) {
                eprintln!(
                    "[ridlc] warning: skipping srv {}/{}: {:#}",
                    package, name, e
                );
            }
        }
        Ok(())
    }

    pub fn resolve_srv(&mut self, package: &str, name: &str) -> Result<()> {
        let key = (package.to_string(), name.to_string());
        if self.srv_cache.contains_key(&key) {
            return Ok(());
        }
        let path = self.find_srv_path(package, name).with_context(|| {
            format!("{RIDLC_ERR_PREFIX} .srv file not found: {package}/srv/{name}")
        })?;
        let spec = parse_srv_file(package, name, &path)?;
        // Resolve all message types referenced in request and response
        for field in spec
            .request
            .fields
            .iter()
            .chain(spec.response.fields.iter())
        {
            if let MsgTypeRef::Named {
                package: pkg,
                name: nm,
            } = &field.type_ref
            {
                let dep_ctx = ResolveContext {
                    namespace: Some(spec.package.clone()),
                    interface_kind: Some("srv"),
                    interface_name: Some(spec.name.clone()),
                    field_name: Some(field.name.clone()),
                };
                self.resolve_named_type(pkg, nm, Some((&ros_msg_type_fmt(pkg, nm), &dep_ctx)))?;
            }
        }
        match &spec.grpc_stream {
            Some(GrpcStreamMode::ServerStream {
                element:
                    MsgTypeRef::Named {
                        package: pkg,
                        name: nm,
                    },
            })
            | Some(GrpcStreamMode::ClientStream {
                element:
                    MsgTypeRef::Named {
                        package: pkg,
                        name: nm,
                    },
            }) => {
                let dep_ctx = ResolveContext {
                    namespace: Some(spec.package.clone()),
                    interface_kind: Some("srv"),
                    interface_name: Some(spec.name.clone()),
                    field_name: Some("<stream_element>".into()),
                };
                self.resolve_named_type(pkg, nm, Some((&ros_msg_type_fmt(pkg, nm), &dep_ctx)))?;
            }
            Some(_) => {
                bail!(
                    "{RIDLC_ERR_PREFIX} stream_server/stream_client element must be a named ROS message type"
                );
            }
            None => {}
        }
        self.srv_cache.insert(key, spec);
        Ok(())
    }

    pub fn find_srv_path(&self, package: &str, name: &str) -> Option<PathBuf> {
        let candidates = package_aliases(package);
        for candidate in candidates {
            if let Some(path) = self
                .srv_index
                .get(&(candidate.to_string(), name.to_string()))
            {
                return Some(path.clone());
            }
        }
        None
    }

    pub fn ordered_srvs(&self) -> Vec<&SrvSpec> {
        let mut keys: Vec<_> = self.srv_cache.keys().cloned().collect();
        keys.sort();
        keys.iter()
            .filter_map(|key| self.srv_cache.get(key))
            .collect::<Vec<_>>()
    }
}

pub fn index_msg_files(root: &Path, index: &mut HashMap<(String, String), PathBuf>) -> Result<()> {
    if !root.exists() {
        return Ok(());
    }
    for entry in fs::read_dir(root).with_context(|| {
        format!(
            "{RIDLC_ERR_PREFIX} failed to read include directory '{}' (check that path exists)",
            root.display()
        )
    })? {
        let entry = entry?;
        let path = entry.path();
        if path.is_dir() {
            index_msg_files(&path, index)?;
            continue;
        }
        if path.extension().and_then(|s| s.to_str()) != Some("msg") {
            continue;
        }
        let Some(stem) = path.file_stem().and_then(|s| s.to_str()) else {
            continue;
        };
        let Some(package) = infer_package_name(&path) else {
            continue;
        };
        index.entry((package, stem.to_string())).or_insert(path);
    }
    Ok(())
}

pub fn index_srv_files(root: &Path, index: &mut HashMap<(String, String), PathBuf>) -> Result<()> {
    if !root.exists() {
        return Ok(());
    }
    for entry in fs::read_dir(root).with_context(|| {
        format!(
            "{RIDLC_ERR_PREFIX} failed to read include directory '{}' (check that path exists)",
            root.display()
        )
    })? {
        let entry = entry?;
        let path = entry.path();
        if path.is_dir() {
            index_srv_files(&path, index)?;
            continue;
        }
        if path.extension().and_then(|s| s.to_str()) != Some("srv") {
            continue;
        }
        let Some(stem) = path.file_stem().and_then(|s| s.to_str()) else {
            continue;
        };
        let Some(package) = infer_srv_package_name(&path) else {
            continue;
        };
        index.entry((package, stem.to_string())).or_insert(path);
    }
    Ok(())
}

pub fn infer_srv_package_name(path: &Path) -> Option<String> {
    let parent = path.parent()?;
    let parent_name = parent.file_name()?.to_str()?;
    if parent_name == "srv" {
        return parent
            .parent()?
            .file_name()?
            .to_str()
            .map(|s| s.to_string());
    }
    Some(parent_name.to_string())
}

pub fn parse_srv_file(package: &str, name: &str, path: &Path) -> Result<SrvSpec> {
    let src = fs::read_to_string(path).with_context(|| {
        format!(
            "{RIDLC_ERR_PREFIX} failed to read .srv file '{}'",
            path.display()
        )
    })?;
    let (body, grpc_stream) = strip_grpc_stream_directive(&src).with_context(|| {
        format!(
            "{RIDLC_ERR_PREFIX} invalid @robonix.grpc directive in '{}'",
            path.display()
        )
    })?;
    let parts: Vec<&str> = body.splitn(2, "---").collect();
    let request_src = parts.first().unwrap_or(&"");
    let response_src = parts.get(1).unwrap_or(&"");

    let request = parse_msg_section(package, &format!("{name}_Request"), request_src)?;
    let response = parse_msg_section(package, &format!("{name}_Response"), response_src)?;

    Ok(SrvSpec {
        package: package.to_string(),
        name: name.to_string(),
        request,
        response,
        grpc_stream,
    })
}

/// If the first line is `# @robonix.grpc stream_server TYPE` or `stream_client TYPE`, strip it and
/// return the remainder plus the stream mode. `TYPE` is `pkg/msg/Name` or `pkg/Name`.
fn strip_grpc_stream_directive(src: &str) -> Result<(&str, Option<GrpcStreamMode>)> {
    let trimmed = src.trim_start();
    let Some(first_line_end) = trimmed.find('\n') else {
        return Ok((src, None));
    };
    let first_line = trimmed[..first_line_end].trim();
    let rest = trimmed[first_line_end + 1..].trim_start();
    let Some(directive) = first_line.strip_prefix("# @robonix.grpc ") else {
        return Ok((src, None));
    };
    let directive = directive.trim();
    let mode = if let Some(typ) = directive.strip_prefix("stream_server ") {
        Some(GrpcStreamMode::ServerStream {
            element: parse_stream_element_type(typ).with_context(|| {
                format!("{RIDLC_ERR_PREFIX} bad stream_server element type: {typ:?}")
            })?,
        })
    } else if let Some(typ) = directive.strip_prefix("stream_client ") {
        Some(GrpcStreamMode::ClientStream {
            element: parse_stream_element_type(typ).with_context(|| {
                format!("{RIDLC_ERR_PREFIX} bad stream_client element type: {typ:?}")
            })?,
        })
    } else {
        bail!("{RIDLC_ERR_PREFIX} unknown @robonix.grpc directive: {directive:?}");
    };
    Ok((rest, mode))
}

fn parse_stream_element_type(typ: &str) -> Result<MsgTypeRef> {
    let t = typ.trim();
    if let Some((pkg, name)) = parse_ridl_type_ref(t) {
        return Ok(MsgTypeRef::Named { package: pkg, name });
    }
    let parts: Vec<&str> = t.split('/').collect();
    if parts.len() == 2 {
        return Ok(MsgTypeRef::Named {
            package: parts[0].to_string(),
            name: parts[1].to_string(),
        });
    }
    bail!(
        "{RIDLC_ERR_PREFIX} expected TYPE like sensor_msgs/msg/Image or sensor_msgs/Image, got {t:?}"
    )
}

fn parse_msg_section(package: &str, name: &str, src: &str) -> Result<MsgSpec> {
    let mut fields = Vec::new();
    for raw_line in src.lines() {
        let line = raw_line.split('#').next().unwrap_or_default().trim();
        if line.is_empty() || line.contains('=') {
            continue;
        }
        let mut parts = line.split_whitespace();
        let Some(raw_type) = parts.next() else {
            continue;
        };
        let Some(raw_name) = parts.next() else {
            continue;
        };
        let (type_ref, is_array) = parse_msg_field_type(package, raw_type)?;
        fields.push(MsgField {
            name: raw_name.to_string(),
            type_ref,
            is_array,
        });
    }
    Ok(MsgSpec {
        package: package.to_string(),
        name: name.to_string(),
        fields,
    })
}

pub fn infer_package_name(path: &Path) -> Option<String> {
    let parent = path.parent()?;
    let parent_name = parent.file_name()?.to_str()?;
    if parent_name == "msg" {
        return parent
            .parent()?
            .file_name()?
            .to_str()
            .map(|s| s.to_string());
    }
    Some(parent_name.to_string())
}

pub fn parse_msg_file(package: &str, name: &str, path: &Path) -> Result<MsgSpec> {
    let src = fs::read_to_string(path).with_context(|| {
        format!(
            "{RIDLC_ERR_PREFIX} failed to read .msg file '{}'",
            path.display()
        )
    })?;
    let mut fields = Vec::new();
    for raw_line in src.lines() {
        let line = raw_line.split('#').next().unwrap_or_default().trim();
        if line.is_empty() || line.contains('=') {
            continue;
        }
        let mut parts = line.split_whitespace();
        let Some(raw_type) = parts.next() else {
            continue;
        };
        let Some(raw_name) = parts.next() else {
            continue;
        };
        let (type_ref, is_array) = parse_msg_field_type(package, raw_type).with_context(|| {
            format!(
                "{RIDLC_ERR_PREFIX} invalid field in '{}' at line with type '{}'",
                path.display(),
                raw_type
            )
        })?;
        fields.push(MsgField {
            name: raw_name.to_string(),
            type_ref,
            is_array,
        });
    }
    Ok(MsgSpec {
        package: package.to_string(),
        name: name.to_string(),
        fields,
    })
}

pub fn parse_msg_field_type(current_package: &str, raw_type: &str) -> Result<(MsgTypeRef, bool)> {
    let normalized = raw_type.split("<=").next().unwrap_or(raw_type).trim();
    let (base_type, is_array) = if let Some(idx) = normalized.find('[') {
        (&normalized[..idx], true)
    } else {
        (normalized, false)
    };
    let base_type = base_type.trim();
    if base_type.is_empty() {
        bail!("{RIDLC_ERR_PREFIX} empty message field type in .msg file");
    }
    if is_ros_primitive(base_type) {
        return Ok((MsgTypeRef::Primitive(base_type.to_string()), is_array));
    }
    if let Some((package, name)) = base_type.split_once('/') {
        return Ok((
            MsgTypeRef::Named {
                package: package.to_string(),
                name: name.to_string(),
            },
            is_array,
        ));
    }
    Ok((
        MsgTypeRef::Named {
            package: current_package.to_string(),
            name: base_type.to_string(),
        },
        is_array,
    ))
}

pub fn is_ros_primitive(raw: &str) -> bool {
    matches!(
        raw,
        "bool"
            | "byte"
            | "uint8"
            | "char"
            | "int8"
            | "float32"
            | "float64"
            | "int16"
            | "uint16"
            | "int32"
            | "uint32"
            | "int64"
            | "uint64"
            | "string"
            | "wstring"
    )
}

pub fn parse_ridl_type_ref(type_ref: &str) -> Option<(String, String)> {
    let trimmed = type_ref.trim();
    let mut parts = trimmed.split('/');
    let package = parts.next()?;
    let middle = parts.next()?;
    let mut name = parts.next()?.to_string();
    if middle != "msg" {
        return None;
    }
    if name.ends_with("[]") {
        name.truncate(name.len().saturating_sub(2));
    }
    Some((package.to_string(), name))
}

pub fn ros_msg_type_fmt(package: &str, name: &str) -> String {
    format!("{package}/msg/{name}")
}

pub fn package_aliases(package: &str) -> Vec<&str> {
    match package {
        "robonix_msgs" => vec!["robonix_msgs", "robonix_msg"],
        _ => vec![package],
    }
}

pub fn format_resolve_error(
    full_type: &str,
    from: Option<(&str, &ResolveContext)>,
    include_paths: &[PathBuf],
) -> String {
    let mut msg = format!(
        "{RIDLC_ERR_PREFIX} failed to resolve ROS message type '{}'",
        full_type
    );
    if let Some((_type_ref, ctx)) = from {
        let mut parts = Vec::new();
        if let Some(ref ns) = ctx.namespace {
            parts.push(format!("namespace '{ns}'"));
        }
        if let (Some(kind), Some(ref name)) = (ctx.interface_kind, ctx.interface_name.as_ref()) {
            parts.push(format!("{kind} '{name}'"));
        }
        if let Some(ref f) = ctx.field_name {
            parts.push(format!("field '{f}'"));
        }
        if !parts.is_empty() {
            msg.push_str(&format!("\n  referenced from: {}", parts.join(", ")));
        }
    }
    msg.push_str("\n  searched include paths:");
    for p in include_paths {
        msg.push_str(&format!("\n    - {}", p.display()));
    }
    msg.push_str("\n  hint: ensure the .msg file exists (e.g. <include>/common_interfaces/std_msgs/msg/Float64.msg)");
    msg
}
