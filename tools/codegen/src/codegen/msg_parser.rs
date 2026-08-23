// SPDX-License-Identifier: MulanPSL-2.0
// Shared ROS .msg / .srv file parser — used by proto_gen, mcp_python_gen,
// and atlas's contract registry.

use anyhow::{Context, Result, bail};
use std::collections::{BTreeSet, HashMap};
use std::fs;
use std::path::{Path, PathBuf};

use super::err::RIDLC_ERR_PREFIX;

#[derive(Clone, Debug)]
pub struct MsgField {
    pub name: String,
    pub type_ref: MsgTypeRef,
    pub is_array: bool,
    /// Fixed array size (e.g. 9 for `float32[9]`). None means unbounded (`T[]`).
    pub array_size: Option<usize>,
    /// Upper bound for `string<=N` / `wstring<=N` fields. None when not
    /// declared. Preserved so MCP JSON Schema can emit `maxLength`.
    pub string_max_len: Option<usize>,
    /// Trailing-comment annotation from the .msg line, e.g.
    /// `# in metres` or `# range [0, 1]`. Empty when there was no
    /// comment. Standard ROS messages use these for units / value
    /// ranges; we propagate them to MCP `description` because they
    /// give an LLM real semantic hints about the field.
    pub description: String,
}

#[derive(Clone, Debug)]
pub struct MsgConstant {
    pub name: String,
    pub type_name: String,
    pub value: i32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MsgTypeRef {
    /// Carries the canonical ROS primitive identifier as a string. Use
    /// `RosPrimitive::parse(...)` to recover the strongly-typed enum
    /// when generating code; the string representation is the source
    /// of truth for cross-generator consistency.
    Primitive(String),
    Named {
        package: String,
        name: String,
    },
}

/// Strongly-typed enum of every ROS2 IDL primitive. **This is the
/// single source of truth for how a primitive maps to wire formats
/// across generators.** Both `proto_gen` and `mcp_python_gen` consume
/// the methods on this enum so a new primitive only has to be added
/// here once.
///
/// The catch-all silent fallbacks that used to live in each generator
/// (`_ => "int"`, `_ => "bytes"`) are gone — every primitive must be
/// listed explicitly. An unrecognised type now panics at codegen time
/// instead of silently producing the wrong wire shape.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RosPrimitive {
    Bool,
    Byte, // ROS2: signed 8-bit. NOT the same as uint8 (unsigned).
    Char, // ROS2: unsigned 8-bit. NOT the same as int8.
    Int8,
    Uint8,
    Int16,
    Uint16,
    Int32,
    Uint32,
    Int64,
    Uint64,
    Float32,
    Float64,
    String,
    Wstring,
}

impl RosPrimitive {
    pub fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "bool" => Self::Bool,
            "byte" => Self::Byte,
            "char" => Self::Char,
            "int8" => Self::Int8,
            "uint8" => Self::Uint8,
            "int16" => Self::Int16,
            "uint16" => Self::Uint16,
            "int32" => Self::Int32,
            "uint32" => Self::Uint32,
            "int64" => Self::Int64,
            "uint64" => Self::Uint64,
            "float32" => Self::Float32,
            "float64" => Self::Float64,
            "string" => Self::String,
            "wstring" => Self::Wstring,
            _ => return None,
        })
    }

    pub fn as_ros_str(&self) -> &'static str {
        match self {
            Self::Bool => "bool",
            Self::Byte => "byte",
            Self::Char => "char",
            Self::Int8 => "int8",
            Self::Uint8 => "uint8",
            Self::Int16 => "int16",
            Self::Uint16 => "uint16",
            Self::Int32 => "int32",
            Self::Uint32 => "uint32",
            Self::Int64 => "int64",
            Self::Uint64 => "uint64",
            Self::Float32 => "float32",
            Self::Float64 => "float64",
            Self::String => "string",
            Self::Wstring => "wstring",
        }
    }

    /// proto3 wire type for this primitive.
    pub fn proto_type(&self) -> &'static str {
        match self {
            Self::Bool => "bool",
            // ROS `byte` is signed 8-bit, ROS `char` is unsigned 8-bit;
            // proto3 has no narrower-than-32-bit ints, so widen but
            // preserve sign.
            Self::Byte | Self::Int8 => "int32",
            Self::Char | Self::Uint8 => "uint32",
            Self::Int16 => "int32",
            Self::Uint16 => "uint32",
            Self::Int32 => "int32",
            Self::Uint32 => "uint32",
            Self::Int64 => "int64",
            Self::Uint64 => "uint64",
            Self::Float32 => "float",
            Self::Float64 => "double",
            Self::String | Self::Wstring => "string",
        }
    }

    /// Python type name used in MCP dataclass annotations.
    pub fn python_type(&self) -> &'static str {
        match self {
            Self::Bool => "bool",
            Self::Float32 | Self::Float64 => "float",
            Self::String | Self::Wstring => "str",
            Self::Byte
            | Self::Char
            | Self::Int8
            | Self::Uint8
            | Self::Int16
            | Self::Uint16
            | Self::Int32
            | Self::Uint32
            | Self::Int64
            | Self::Uint64 => "int",
        }
    }

    /// Default literal in Python for a non-array field.
    pub fn python_default(&self) -> &'static str {
        match self {
            Self::Bool => "False",
            Self::Float32 | Self::Float64 => "0.0",
            Self::String | Self::Wstring => "\"\"",
            _ => "0",
        }
    }

    /// Python cast invoked by `from_dict()` when validating an inbound
    /// JSON value. Same as the type name for builtin casts.
    pub fn python_cast(&self) -> &'static str {
        self.python_type()
    }

    /// JSON Schema base type ("integer" / "number" / "boolean" / "string").
    pub fn json_schema_type(&self) -> &'static str {
        match self {
            Self::Bool => "boolean",
            Self::Float32 | Self::Float64 => "number",
            Self::String | Self::Wstring => "string",
            _ => "integer",
        }
    }

    /// Inclusive range constraint for integer primitives (used by
    /// `from_dict` validation and JSON Schema `minimum`/`maximum`).
    /// Floats / strings / bool return None — JSON Schema's defaults
    /// already cover them.
    pub fn integer_range(&self) -> Option<(i128, i128)> {
        Some(match self {
            Self::Byte | Self::Int8 => (i8::MIN as i128, i8::MAX as i128),
            Self::Char | Self::Uint8 => (0, u8::MAX as i128),
            Self::Int16 => (i16::MIN as i128, i16::MAX as i128),
            Self::Uint16 => (0, u16::MAX as i128),
            Self::Int32 => (i32::MIN as i128, i32::MAX as i128),
            Self::Uint32 => (0, u32::MAX as i128),
            Self::Int64 => (i64::MIN as i128, i64::MAX as i128),
            // uint64 max overflows i128's signed range only in the
            // very upper bit; i128 holds it fine.
            Self::Uint64 => (0, u64::MAX as i128),
            _ => return None,
        })
    }

    /// True if this primitive is the canonical "raw byte buffer" type.
    /// Only `uint8` qualifies — `byte` is signed 8-bit and shouldn't
    /// be conflated. Used by both proto and MCP gens to decide whether
    /// a `T[]` field becomes a wire-level byte blob (`bytes` in proto,
    /// base64-encoded `bytes` in MCP).
    pub fn is_blob_element(&self) -> bool {
        matches!(self, Self::Uint8)
    }
}

#[derive(Clone, Debug)]
pub struct MsgSpec {
    pub package: String,
    pub name: String,
    pub constants: Vec<MsgConstant>,
    pub fields: Vec<MsgField>,
}

#[derive(Clone, Debug)]
pub struct SrvSpec {
    pub package: String,
    pub name: String,
    pub request: MsgSpec,
    pub response: MsgSpec,
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
        let mut visiting: BTreeSet<(String, String)> = BTreeSet::new();
        self.resolve_named_type_inner(package, name, from, &mut visiting)
    }

    /// Inner recursive resolver that detects cycles. A self-referential
    /// or mutually-recursive .msg used to stack-overflow here; now we
    /// short-circuit on revisit (the dependency closure is already
    /// being satisfied by an outer frame, so the second visit can be a
    /// no-op without losing the field list).
    fn resolve_named_type_inner(
        &mut self,
        package: &str,
        name: &str,
        from: Option<(&str, &ResolveContext)>,
        visiting: &mut BTreeSet<(String, String)>,
    ) -> Result<()> {
        let key = (package.to_string(), name.to_string());
        if self.cache.contains_key(&key) {
            return Ok(());
        }
        if !visiting.insert(key.clone()) {
            // Cycle: we're already resolving this type higher up the
            // call stack. Bail without re-parsing — the outer frame
            // will populate the cache once it returns.
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
                self.resolve_named_type_inner(
                    package,
                    name,
                    Some((&ros_msg_type_fmt(package, name), &dep_ctx)),
                    visiting,
                )?;
            }
        }
        visiting.remove(&key);
        self.cache.insert(key, spec);
        Ok(())
    }

    pub fn resolve_all_in_index(&mut self, verbose: bool, skip_count: &mut usize) -> Result<()> {
        let keys: Vec<_> = self.index.keys().cloned().collect();
        for (package, name) in keys {
            if let Err(e) = self.resolve_named_type(&package, &name, None) {
                if verbose {
                    eprintln!(
                        "[robonix-codegen] warning: skipping {}/{}: {:#}",
                        package, name, e
                    );
                } else {
                    *skip_count += 1;
                }
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

    pub fn resolve_all_srv(&mut self, verbose: bool, skip_count: &mut usize) -> Result<()> {
        let keys: Vec<_> = self.srv_index.keys().cloned().collect();
        for (package, name) in keys {
            if let Err(e) = self.resolve_srv(&package, &name) {
                if verbose {
                    eprintln!(
                        "[robonix-codegen] warning: skipping srv {}/{}: {:#}",
                        package, name, e
                    );
                } else {
                    *skip_count += 1;
                }
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

    pub fn srv_spec(&self, package: &str, name: &str) -> Option<&SrvSpec> {
        self.srv_cache.get(&(package.to_string(), name.to_string()))
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

/// Strip legacy `# @robonix.grpc ...` lines from the top of `.srv` files.
fn strip_leading_robonix_grpc_lines(src: &str) -> String {
    let mut s = src.to_string();
    loop {
        let t = s.trim_start();
        let first = t.lines().next().unwrap_or("").trim();
        if first.starts_with("# @robonix.grpc") {
            if let Some(pos) = s.find('\n') {
                s = s[pos + 1..].to_string();
                continue;
            }
            s.clear();
            break;
        }
        break;
    }
    s
}

pub fn parse_srv_file(package: &str, name: &str, path: &Path) -> Result<SrvSpec> {
    let src = fs::read_to_string(path).with_context(|| {
        format!(
            "{RIDLC_ERR_PREFIX} failed to read .srv file '{}'",
            path.display()
        )
    })?;
    let body = strip_leading_robonix_grpc_lines(&src);

    // Split on a line that's exactly `---` after trimming. The
    // previous `splitn(2, "---")` matched `---` as a substring, so a
    // comment like `# --- separator` would split mid-comment and
    // wreck the response section. Line-based split avoids that.
    let mut request_lines: Vec<&str> = Vec::new();
    let mut response_lines: Vec<&str> = Vec::new();
    let mut in_response = false;
    for line in body.lines() {
        if !in_response && line.trim() == "---" {
            in_response = true;
            continue;
        }
        if in_response {
            response_lines.push(line);
        } else {
            request_lines.push(line);
        }
    }
    let request_src = request_lines.join("\n");
    let response_src = response_lines.join("\n");

    let request = parse_msg_section(package, &format!("{name}_Request"), &request_src)
        .with_context(|| format!("{RIDLC_ERR_PREFIX} parsing request of '{}'", path.display()))?;
    let response = parse_msg_section(package, &format!("{name}_Response"), &response_src)
        .with_context(|| {
            format!(
                "{RIDLC_ERR_PREFIX} parsing response of '{}'",
                path.display()
            )
        })?;

    Ok(SrvSpec {
        package: package.to_string(),
        name: name.to_string(),
        request,
        response,
    })
}

fn parse_msg_section(package: &str, name: &str, src: &str) -> Result<MsgSpec> {
    let (constants, fields) = parse_msg_members_from_lines(package, src, None)?;
    Ok(MsgSpec {
        package: package.to_string(),
        name: name.to_string(),
        constants,
        fields,
    })
}

/// Single member-parsing pass shared between `parse_msg_file` and
/// `parse_msg_section` (for srv request / response bodies). Captures:
///
///   - integer constants (`int32 FOO = 42`) as first-class entries for proto enum generation
///   - trailing comments (`float64 x  # in metres`) → `field.description`
///   - `string<=N` upper bounds → `field.string_max_len`
///
/// `path_hint` is used purely for error messages.
fn parse_msg_members_from_lines(
    package: &str,
    src: &str,
    path_hint: Option<&Path>,
) -> Result<(Vec<MsgConstant>, Vec<MsgField>)> {
    let mut constants = Vec::new();
    let mut fields = Vec::new();
    for raw_line in src.lines() {
        // Split off the trailing comment (if any). The comment text
        // is preserved as `description` so MCP JSON Schema can show
        // it to the LLM.
        let (code, comment) = match raw_line.find('#') {
            Some(idx) => (&raw_line[..idx], raw_line[idx + 1..].trim()),
            None => (raw_line, ""),
        };
        let code = code.trim();
        if code.is_empty() {
            continue;
        }
        if let Some(constant) = parse_constant_line(code).with_context(|| match path_hint {
            Some(p) => format!(
                "{RIDLC_ERR_PREFIX} invalid constant in '{}' at line '{}'",
                p.display(),
                code
            ),
            None => format!("{RIDLC_ERR_PREFIX} invalid constant at line '{code}'"),
        })? {
            constants.push(constant);
            continue;
        }
        let mut parts = code.split_whitespace();
        let Some(raw_type) = parts.next() else {
            continue;
        };
        let Some(raw_name) = parts.next() else {
            continue;
        };
        let (type_ref, is_array, array_size, string_max_len) =
            parse_msg_field_type(package, raw_type).with_context(|| match path_hint {
                Some(p) => format!(
                    "{RIDLC_ERR_PREFIX} invalid field in '{}' at line with type '{}'",
                    p.display(),
                    raw_type
                ),
                None => format!(
                    "{RIDLC_ERR_PREFIX} invalid field at line with type '{}'",
                    raw_type
                ),
            })?;
        fields.push(MsgField {
            name: raw_name.to_string(),
            type_ref,
            is_array,
            array_size,
            string_max_len,
            description: comment.to_string(),
        });
    }
    Ok((constants, fields))
}

/// Parse ROS integer constants such as `uint32 NODE_STATE=1`.
///
/// Non-constant lines return `Ok(None)`. String and floating-point constants
/// are intentionally ignored because protobuf enum values must be integers.
fn parse_constant_line(code: &str) -> Result<Option<MsgConstant>> {
    let Some((lhs, rhs)) = code.split_once('=') else {
        return Ok(None);
    };
    let mut parts = lhs.split_whitespace();
    let Some(type_name) = parts.next() else {
        return Ok(None);
    };
    let Some(name) = parts.next() else {
        return Ok(None);
    };
    let Some(primitive) = RosPrimitive::parse(type_name) else {
        return Ok(None);
    };
    if !matches!(
        primitive,
        RosPrimitive::Byte
            | RosPrimitive::Char
            | RosPrimitive::Int8
            | RosPrimitive::Uint8
            | RosPrimitive::Int16
            | RosPrimitive::Uint16
            | RosPrimitive::Int32
            | RosPrimitive::Uint32
            | RosPrimitive::Int64
            | RosPrimitive::Uint64
    ) {
        return Ok(None);
    }
    let value_text = rhs.split_whitespace().next().unwrap_or("").trim();
    let value = value_text
        .parse::<i32>()
        .with_context(|| format!("constant '{name}' value '{value_text}' is not an i32"))?;
    Ok(Some(MsgConstant {
        name: name.to_string(),
        type_name: type_name.to_string(),
        value,
    }))
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
    let (constants, fields) = parse_msg_members_from_lines(package, &src, Some(path))?;
    Ok(MsgSpec {
        package: package.to_string(),
        name: name.to_string(),
        constants,
        fields,
    })
}

/// Parse the type-side of one .msg field line. Returns
/// `(type_ref, is_array, array_size, string_max_len)`.
///
/// `string_max_len` carries the `<=N` upper bound on `string<=N` /
/// `wstring<=N` declarations (preserved into MCP JSON Schema as
/// `maxLength`). It only applies to the string primitives.
pub fn parse_msg_field_type(
    current_package: &str,
    raw_type: &str,
) -> Result<(MsgTypeRef, bool, Option<usize>, Option<usize>)> {
    // Capture string-bound `<=N` BEFORE stripping it, so we can
    // surface it on MsgField. Only meaningful for string/wstring;
    // other types ignore it.
    let (without_bound, bound) = match raw_type.split_once("<=") {
        Some((lhs, rhs)) => {
            let n = rhs
                .trim_matches(|c: char| c.is_whitespace() || c == ']' || c == '[')
                .split(|c: char| !c.is_ascii_digit())
                .next()
                .unwrap_or("")
                .parse::<usize>()
                .ok();
            (lhs.trim(), n)
        }
        None => (raw_type.trim(), None),
    };
    let normalized = without_bound;
    let (base_type, is_array, array_size) = if let Some(idx) = normalized.find('[') {
        let bracket = &normalized[idx..]; // e.g. "[]" or "[9]"
        let size = bracket
            .trim_matches(|c| c == '[' || c == ']')
            .parse::<usize>()
            .ok(); // None for "[]", Some(N) for "[N]"
        (&normalized[..idx], true, size)
    } else {
        (normalized, false, None)
    };
    let base_type = base_type.trim();
    if base_type.is_empty() {
        bail!("{RIDLC_ERR_PREFIX} empty message field type in .msg file");
    }
    let string_max_len = match base_type {
        "string" | "wstring" => bound,
        _ => None,
    };
    if RosPrimitive::parse(base_type).is_some() {
        return Ok((
            MsgTypeRef::Primitive(base_type.to_string()),
            is_array,
            array_size,
            string_max_len,
        ));
    }
    // `pkg/msg/TypeName` (ROS fully-qualified message type)
    let segs: Vec<&str> = base_type.split('/').collect();
    if segs.len() == 3 && segs[1] == "msg" {
        let pkg = segs[0].trim();
        let nm = segs[2].trim();
        if pkg.is_empty() || nm.is_empty() {
            bail!(
                "{RIDLC_ERR_PREFIX} invalid pkg/msg/Name reference '{}': empty package or name",
                base_type
            );
        }
        return Ok((
            MsgTypeRef::Named {
                package: pkg.to_string(),
                name: nm.to_string(),
            },
            is_array,
            array_size,
            string_max_len,
        ));
    }
    if let Some((package, name)) = base_type.split_once('/') {
        let pkg = package.trim();
        let nm = name.trim();
        if pkg.is_empty() || nm.is_empty() {
            bail!(
                "{RIDLC_ERR_PREFIX} invalid pkg/Name reference '{}': empty package or name",
                base_type
            );
        }
        return Ok((
            MsgTypeRef::Named {
                package: pkg.to_string(),
                name: nm.to_string(),
            },
            is_array,
            array_size,
            string_max_len,
        ));
    }
    Ok((
        MsgTypeRef::Named {
            package: current_package.to_string(),
            name: base_type.to_string(),
        },
        is_array,
        array_size,
        string_max_len,
    ))
}

/// Back-compat alias around the canonical `RosPrimitive::parse`. Kept
/// because external callers may still spell it this way.
pub fn is_ros_primitive(raw: &str) -> bool {
    RosPrimitive::parse(raw).is_some()
}

/// Parse a fully-qualified IDL type ref of the form `pkg/msg/Name` or
/// `pkg/srv/Name`. Returns `(package, name)` — the middle segment is
/// dropped after validation. Use `parse_qualified_type_ref` if the
/// caller needs to know whether it was a msg or srv reference.
///
/// Trailing `[]` (array-ness in TOML refs) is stripped from the name.
pub fn parse_ridl_type_ref(type_ref: &str) -> Option<(String, String)> {
    parse_qualified_type_ref(type_ref).map(|(_, p, n)| (p, n))
}

/// Like `parse_ridl_type_ref` but also returns whether the middle was
/// `msg` or `srv`. Returns `None` for any other middle segment or for
/// shapes that aren't `pkg/<msg|srv>/Name`.
pub fn parse_qualified_type_ref(type_ref: &str) -> Option<(IdlKind, String, String)> {
    let trimmed = type_ref.trim();
    let mut parts = trimmed.split('/');
    let package = parts.next()?;
    let middle = parts.next()?;
    let mut name = parts.next()?.to_string();
    if parts.next().is_some() {
        return None;
    }
    let kind = match middle {
        "msg" => IdlKind::Msg,
        "srv" => IdlKind::Srv,
        _ => return None,
    };
    if package.is_empty() || name.is_empty() {
        return None;
    }
    if name.ends_with("[]") {
        name.truncate(name.len().saturating_sub(2));
    }
    Some((kind, package.to_string(), name))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IdlKind {
    Msg,
    Srv,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_idl_tree_resolves_ros_action_interfaces() -> Result<()> {
        let include = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../capabilities/lib")
            .canonicalize()?;
        let mut resolver = MsgResolver::new(&[include])?;
        let mut skipped = 0;

        resolver.resolve_all_in_index(false, &mut skipped)?;
        resolver.resolve_all_srv(false, &mut skipped)?;

        assert_eq!(skipped, 0, "the canonical IDL tree must resolve completely");
        assert!(
            resolver
                .cache
                .contains_key(&("unique_identifier_msgs".to_string(), "UUID".to_string()))
        );
        assert!(
            resolver
                .cache
                .contains_key(&("action_msgs".to_string(), "GoalStatusArray".to_string()))
        );
        assert!(
            resolver
                .srv_cache
                .contains_key(&("action_msgs".to_string(), "CancelGoal".to_string()))
        );
        Ok(())
    }
}
