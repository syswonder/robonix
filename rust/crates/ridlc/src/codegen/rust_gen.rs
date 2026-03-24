// SPDX-License-Identifier: MulanPSL-2.0

use crate::ast::{Annotation, CommandDef, File, Interface, QueryDef, StreamDef, StreamDirection};
use anyhow::{Context, Result, bail};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

use super::msg_parser::{MsgResolver, MsgSpec, MsgTypeRef, ResolveContext, parse_ridl_type_ref};
use crate::codegen::err::RIDLC_ERR_PREFIX;

const RUNTIME_CRATE_ROOT: &str = "robonix_server";

fn map_ros_to_rust_primitive(raw: &str) -> &'static str {
    match raw {
        "bool" => "bool",
        "byte" | "uint8" => "u8",
        "char" | "int8" => "i8",
        "float32" => "f32",
        "float64" => "f64",
        "int16" => "i16",
        "uint16" => "u16",
        "int32" => "i32",
        "uint32" => "u32",
        "int64" => "i64",
        "uint64" => "u64",
        "string" | "wstring" => "std::string::String",
        _ => "u8",
    }
}

fn sanitize_field_name(name: &str) -> String {
    if is_rust_keyword(name) {
        format!("r#{name}")
    } else {
        name.to_string()
    }
}

fn sanitize_module_name(name: &str) -> String {
    name.replace('/', "_")
}

fn pascal(name: &str) -> String {
    name.split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|part| !part.is_empty())
        .map(|part| {
            let mut chars = part.chars();
            let mut out = String::new();
            if let Some(first) = chars.next() {
                out.push(first.to_ascii_uppercase());
                for ch in chars {
                    out.push(ch.to_ascii_lowercase());
                }
            }
            out
        })
        .collect()
}

fn namespace_type_prefix(ns: &str) -> String {
    ns.split('/')
        .filter(|part| !part.is_empty() && *part != "robonix")
        .map(pascal)
        .collect::<String>()
}

fn rosidl_symbol_name(ns: &str, iface_name: &str) -> String {
    format!("{}{}", namespace_type_prefix(ns), pascal(iface_name))
}

fn is_rust_keyword(name: &str) -> bool {
    matches!(
        name,
        "as" | "break"
            | "const"
            | "continue"
            | "crate"
            | "else"
            | "enum"
            | "extern"
            | "false"
            | "fn"
            | "for"
            | "if"
            | "impl"
            | "in"
            | "let"
            | "loop"
            | "match"
            | "mod"
            | "move"
            | "mut"
            | "pub"
            | "ref"
            | "return"
            | "self"
            | "Self"
            | "static"
            | "struct"
            | "super"
            | "trait"
            | "true"
            | "type"
            | "unsafe"
            | "use"
            | "where"
            | "while"
            | "async"
            | "await"
            | "dyn"
    )
}

fn rust_type_expr(ty: &MsgTypeRef, is_array: bool) -> String {
    let base = match ty {
        MsgTypeRef::Primitive(p) => map_ros_to_rust_primitive(p).to_string(),
        MsgTypeRef::Named { package, name } => {
            format!(
                "{RUNTIME_CRATE_ROOT}::generated::types::{}::{}",
                package, name
            )
        }
    };
    if is_array {
        format!("Vec<{base}>")
    } else {
        base
    }
}

fn push_interface_imports(
    namespace: &str,
    ast: &File,
    resolver: &mut MsgResolver,
) -> Result<()> {
    for iface in &ast.interfaces {
        match iface {
            Interface::Query(q) => {
                let ctx = ResolveContext {
                    namespace: Some(namespace.to_string()),
                    interface_kind: Some("query"),
                    interface_name: Some(q.name.clone()),
                    field_name: Some(q.request.name.clone()),
                };
                resolver.resolve_ridl_type(&q.request.type_ref, &ctx)?;
                let ctx = ResolveContext {
                    namespace: Some(namespace.to_string()),
                    interface_kind: Some("query"),
                    interface_name: Some(q.name.clone()),
                    field_name: Some(q.response.name.clone()),
                };
                resolver.resolve_ridl_type(&q.response.type_ref, &ctx)?;
            }
            Interface::Stream(s) => {
                for field in &s.fields {
                    let ctx = ResolveContext {
                        namespace: Some(namespace.to_string()),
                        interface_kind: Some("stream"),
                        interface_name: Some(s.name.clone()),
                        field_name: Some(field.name.clone()),
                    };
                    resolver.resolve_ridl_type(&field.type_ref, &ctx)?;
                }
            }
            Interface::Command(c) => {
                if let Some(field) = &c.input {
                    let ctx = ResolveContext {
                        namespace: Some(namespace.to_string()),
                        interface_kind: Some("command"),
                        interface_name: Some(c.name.clone()),
                        field_name: Some(field.name.clone()),
                    };
                    resolver.resolve_ridl_type(&field.type_ref, &ctx)?;
                }
                if let Some(field) = &c.output {
                    let ctx = ResolveContext {
                        namespace: Some(namespace.to_string()),
                        interface_kind: Some("command"),
                        interface_name: Some(c.name.clone()),
                        field_name: Some(field.name.clone()),
                    };
                    resolver.resolve_ridl_type(&field.type_ref, &ctx)?;
                }
                if let Some(field) = &c.result {
                    let ctx = ResolveContext {
                        namespace: Some(namespace.to_string()),
                        interface_kind: Some("command"),
                        interface_name: Some(c.name.clone()),
                        field_name: Some(field.name.clone()),
                    };
                    resolver.resolve_ridl_type(&field.type_ref, &ctx)?;
                }
            }
        }
    }
    Ok(())
}

pub fn generate_bindings(
    files_by_ns: &BTreeMap<String, File>,
    include_paths: &[PathBuf],
    out_file: &Path,
) -> Result<()> {
    let mut resolver = MsgResolver::new(include_paths)?;
    for (namespace, ast) in files_by_ns {
        push_interface_imports(namespace, ast, &mut resolver)?;
    }

    let mut out = String::new();
    out.push_str("// @generated by ridlc --lang rust\n\n");
    out.push_str("pub mod types {\n");

    let mut packages = BTreeSet::new();
    for spec in resolver.ordered_specs() {
        packages.insert(spec.package.clone());
    }
    for package in packages {
        out.push_str(&format!("    pub mod {} {{\n", package));
        if package == "std_msgs" {
            emit_std_msgs_module(&mut out, "        ")?;
        }
        for spec in resolver
            .ordered_specs()
            .into_iter()
            .filter(|spec| spec.package == package)
        {
            if package == "std_msgs" && spec.name == "String" {
                continue;
            }
            emit_plain_msg(&mut out, "        ", spec);
        }
        out.push_str("    }\n");
    }
    out.push_str("}\n\n");

    out.push_str("#[derive(Debug, Clone, Copy, PartialEq, Eq)]\n");
    out.push_str("pub enum InterfaceKind { Stream, Command, Query }\n\n");
    out.push_str("#[derive(Debug, Clone, Copy)]\n");
    out.push_str("pub struct InterfaceDescriptor {\n");
    out.push_str("    pub kind: InterfaceKind,\n");
    out.push_str("    pub namespace: &'static str,\n");
    out.push_str("    pub name: &'static str,\n");
    out.push_str("    pub request_type: Option<&'static str>,\n");
    out.push_str("    pub response_type: Option<&'static str>,\n");
    out.push_str("    pub input_type: Option<&'static str>,\n");
    out.push_str("    pub output_type: Option<&'static str>,\n");
    out.push_str("    pub result_type: Option<&'static str>,\n");
    out.push_str("    pub payload_type: Option<&'static str>,\n");
    out.push_str("    pub stream_direction: Option<&'static str>,\n");
    out.push_str("    pub annotations: &'static str,\n");
    out.push_str("}\n\n");
    out.push_str("pub const ALL_INTERFACES: &[InterfaceDescriptor] = &[\n");
    for (namespace, ast) in files_by_ns {
        for iface in &ast.interfaces {
            match iface {
                Interface::Query(q) => push_query_descriptor(&mut out, namespace, q),
                Interface::Stream(s) => push_stream_descriptor(&mut out, namespace, s),
                Interface::Command(c) => push_command_descriptor(&mut out, namespace, c),
            }
        }
    }
    out.push_str("];\n\n");

    for (namespace, ast) in files_by_ns {
        emit_namespace_modules(&mut out, namespace, ast)?;
    }

    let should_write = match fs::read_to_string(out_file) {
        Ok(existing) => existing != out,
        Err(_) => true,
    };
    if should_write {
        if let Some(parent) = out_file.parent() {
            fs::create_dir_all(parent).with_context(|| {
                format!(
                    "{RIDLC_ERR_PREFIX} failed to create output directory '{}'",
                    parent.display()
                )
            })?;
        }
        fs::write(out_file, out).with_context(|| {
            format!(
                "{RIDLC_ERR_PREFIX} failed to write generated Rust bindings to '{}'",
                out_file.display()
            )
        })?;
    }
    Ok(())
}

fn emit_plain_msg(out: &mut String, indent: &str, spec: &MsgSpec) {
    out.push_str(&format!(
        "{indent}#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, Default)]\n"
    ));
    out.push_str(&format!("{indent}pub struct {} {{\n", spec.name));
    for field in &spec.fields {
        out.push_str(&format!(
            "{indent}    pub {}: {},\n",
            sanitize_field_name(&field.name),
            rust_type_expr(&field.type_ref, field.is_array)
        ));
    }
    out.push_str(&format!("{indent}}}\n\n"));
}

fn emit_std_msgs_module(out: &mut String, indent: &str) -> Result<()> {
    out.push_str(&format!(
        "{indent}#[derive(Clone, Debug, PartialEq, PartialOrd, serde::Serialize, serde::Deserialize)]\n"
    ));
    out.push_str(&format!("{indent}pub struct String {{\n"));
    out.push_str(&format!("{indent}    pub data: std::string::String,\n"));
    out.push_str(&format!("{indent}}}\n\n"));
    out.push_str(&format!("{indent}impl Default for String {{\n"));
    out.push_str(&format!("{indent}    fn default() -> Self {{\n"));
    out.push_str(&format!(
        "{indent}        <Self as rosidl_runtime_rs::Message>::from_rmw_message(rmw::String::default())\n"
    ));
    out.push_str(&format!("{indent}    }}\n"));
    out.push_str(&format!("{indent}}}\n\n"));
    out.push_str(&format!(
        "{indent}impl rosidl_runtime_rs::Message for String {{\n"
    ));
    out.push_str(&format!("{indent}    type RmwMsg = rmw::String;\n"));
    out.push_str(&format!(
        "{indent}    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {{\n"
    ));
    out.push_str(&format!("{indent}        match msg_cow {{\n"));
    out.push_str(&format!(
        "{indent}            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {{ data: msg.data.as_str().into() }}),\n"
    ));
    out.push_str(&format!(
        "{indent}            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {{ data: msg.data.as_str().into() }}),\n"
    ));
    out.push_str(&format!("{indent}        }}\n"));
    out.push_str(&format!("{indent}    }}\n"));
    out.push_str(&format!(
        "{indent}    fn from_rmw_message(msg: Self::RmwMsg) -> Self {{ Self {{ data: msg.data.to_string() }} }}\n"
    ));
    out.push_str(&format!("{indent}}}\n\n"));

    out.push_str(&format!("{indent}pub mod rmw {{\n"));
    out.push_str(&format!(
        "{indent}    #[link(name = \"std_msgs__rosidl_typesupport_c\")]\n"
    ));
    out.push_str(&format!("{indent}    unsafe extern \"C\" {{\n"));
    out.push_str(&format!(
        "{indent}        fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__String() -> *const std::ffi::c_void;\n"
    ));
    out.push_str(&format!("{indent}    }}\n\n"));
    out.push_str(&format!(
        "{indent}    #[link(name = \"std_msgs__rosidl_generator_c\")]\n"
    ));
    out.push_str(&format!("{indent}    unsafe extern \"C\" {{\n"));
    out.push_str(&format!(
        "{indent}        fn std_msgs__msg__String__init(msg: *mut String) -> bool;\n"
    ));
    out.push_str(&format!("{indent}    }}\n\n"));
    out.push_str(&format!("{indent}    #[repr(C)]\n"));
    out.push_str(&format!(
        "{indent}    #[derive(Clone, Debug, PartialEq, PartialOrd)]\n"
    ));
    out.push_str(&format!("{indent}    pub struct String {{\n"));
    out.push_str(&format!(
        "{indent}        pub data: rosidl_runtime_rs::String,\n"
    ));
    out.push_str(&format!("{indent}    }}\n\n"));
    out.push_str(&format!("{indent}    impl Default for String {{\n"));
    out.push_str(&format!("{indent}        fn default() -> Self {{\n"));
    out.push_str(&format!("{indent}            unsafe {{\n"));
    out.push_str(&format!(
        "{indent}                let mut msg = std::mem::zeroed();\n"
    ));
    out.push_str(&format!(
        "{indent}                if !std_msgs__msg__String__init(&mut msg as *mut _) {{ panic!(\"Call to std_msgs__msg__String__init() failed\"); }}\n"
    ));
    out.push_str(&format!("{indent}                msg\n"));
    out.push_str(&format!("{indent}            }}\n"));
    out.push_str(&format!("{indent}        }}\n"));
    out.push_str(&format!("{indent}    }}\n\n"));
    out.push_str(&format!(
        "{indent}    impl rosidl_runtime_rs::Message for String {{\n"
    ));
    out.push_str(&format!("{indent}        type RmwMsg = Self;\n"));
    out.push_str(&format!(
        "{indent}        fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {{ msg_cow }}\n"
    ));
    out.push_str(&format!(
        "{indent}        fn from_rmw_message(msg: Self::RmwMsg) -> Self {{ msg }}\n"
    ));
    out.push_str(&format!("{indent}    }}\n"));
    out.push_str(&format!(
        "{indent}    impl rosidl_runtime_rs::RmwMessage for String {{\n"
    ));
    out.push_str(&format!(
        "{indent}        const TYPE_NAME: &'static str = \"std_msgs/msg/String\";\n"
    ));
    out.push_str(&format!(
        "{indent}        fn get_type_support() -> *const std::ffi::c_void {{ unsafe {{ rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__String() }} }}\n"
    ));
    out.push_str(&format!("{indent}    }}\n"));
    out.push_str(&format!("{indent}}}\n\n"));
    Ok(())
}

fn push_query_descriptor(out: &mut String, namespace: &str, q: &QueryDef) {
    let ann = format_annotations_for_descriptor(&q.annotations);
    out.push_str("    InterfaceDescriptor {\n");
    out.push_str("        kind: InterfaceKind::Query,\n");
    out.push_str(&format!("        namespace: \"{}\",\n", namespace));
    out.push_str(&format!("        name: \"{}\",\n", q.name));
    out.push_str(&format!(
        "        request_type: Some(\"{}\"),\n",
        q.request.type_ref
    ));
    out.push_str(&format!(
        "        response_type: Some(\"{}\"),\n",
        q.response.type_ref
    ));
    out.push_str("        input_type: None,\n        output_type: None,\n        result_type: None,\n        payload_type: None,\n        stream_direction: None,\n");
    out.push_str(&format!("        annotations: \"{}\",\n", ann));
    out.push_str("    },\n");
}

fn push_stream_descriptor(out: &mut String, namespace: &str, s: &StreamDef) {
    let first = s.fields.first();
    let payload_type = first.map(|f| f.type_ref.as_str()).unwrap_or("");
    let direction = first.map(|f| match f.direction {
        StreamDirection::Input => "input",
        StreamDirection::Output => "output",
    });
    let ann = format_annotations_for_descriptor(&s.annotations);
    out.push_str("    InterfaceDescriptor {\n");
    out.push_str("        kind: InterfaceKind::Stream,\n");
    out.push_str(&format!("        namespace: \"{}\",\n", namespace));
    out.push_str(&format!("        name: \"{}\",\n", s.name));
    out.push_str("        request_type: None,\n        response_type: None,\n        input_type: None,\n        output_type: None,\n        result_type: None,\n");
    out.push_str(&format!(
        "        payload_type: Some(\"{}\"),\n",
        payload_type
    ));
    out.push_str(&format!(
        "        stream_direction: Some(\"{}\"),\n",
        direction.unwrap_or("unknown")
    ));
    out.push_str(&format!("        annotations: \"{}\",\n", ann));
    out.push_str("    },\n");
}

fn push_command_descriptor(out: &mut String, namespace: &str, c: &CommandDef) {
    let ann = format_annotations_for_descriptor(&c.annotations);
    out.push_str("    InterfaceDescriptor {\n");
    out.push_str("        kind: InterfaceKind::Command,\n");
    out.push_str(&format!("        namespace: \"{}\",\n", namespace));
    out.push_str(&format!("        name: \"{}\",\n", c.name));
    out.push_str("        request_type: None,\n        response_type: None,\n");
    out.push_str(&format!(
        "        input_type: {},\n",
        optional_str(c.input.as_ref().map(|f| f.type_ref.as_str()))
    ));
    out.push_str(&format!(
        "        output_type: {},\n",
        optional_str(c.output.as_ref().map(|f| f.type_ref.as_str()))
    ));
    out.push_str(&format!(
        "        result_type: {},\n",
        optional_str(c.result.as_ref().map(|f| f.type_ref.as_str()))
    ));
    out.push_str("        payload_type: None,\n        stream_direction: None,\n");
    out.push_str(&format!("        annotations: \"{}\",\n", ann));
    out.push_str("    },\n");
}

fn optional_str(value: Option<&str>) -> String {
    match value {
        Some(v) => format!("Some(\"{}\")", v),
        None => "None".to_string(),
    }
}

/// Format interface annotations for the descriptor string.
/// Format: "key" or "key=value" (value escaped for Rust string literal), comma-separated.
fn format_annotations_for_descriptor(annotations: &[Annotation]) -> String {
    if annotations.is_empty() {
        return String::new();
    }
    let mut parts = Vec::new();
    for a in annotations {
        if let Some(ref v) = a.value {
            let mut escaped = String::new();
            for c in v.chars() {
                match c {
                    '\\' => escaped.push_str("\\\\"),
                    '"' => escaped.push_str("\\\""),
                    c => escaped.push(c),
                }
            }
            parts.push(format!("{}={}", a.key, escaped));
        } else {
            parts.push(a.key.clone());
        }
    }
    parts.join(",")
}

fn emit_namespace_modules(out: &mut String, namespace: &str, ast: &File) -> Result<()> {
    let ns_mod = sanitize_module_name(namespace);
    out.push_str(&format!("pub mod {} {{\n", ns_mod));
    let base_indent = "    ";
    for iface in &ast.interfaces {
        match iface {
            Interface::Query(q) => {
                out.push_str(&format!("{base_indent}pub mod {}_query {{\n", q.name));
                emit_query_module(out, namespace, q, "        ")?;
                out.push_str(&format!("{base_indent}}}\n"));
            }
            Interface::Stream(s) => {
                let field = s.fields.first().with_context(|| {
                    format!(
                        "{RIDLC_ERR_PREFIX} stream '{}' in namespace '{}' has no fields (at least one field required)",
                        s.name, namespace
                    )
                })?;
                let direction = match field.direction {
                    StreamDirection::Input => "input",
                    StreamDirection::Output => "output",
                };
                out.push_str(&format!("{base_indent}pub mod {}_stream {{\n", s.name));
                out.push_str(&format!(
                    "{base_indent}    pub type Payload = {};\n",
                    ridl_rust_type(&field.type_ref)?
                ));
                let ann = format_annotations_for_descriptor(&s.annotations);
                out.push_str(&format!(
                    "{base_indent}    pub const DESCRIPTOR: {RUNTIME_CRATE_ROOT}::generated::InterfaceDescriptor = {RUNTIME_CRATE_ROOT}::generated::InterfaceDescriptor {{ kind: {RUNTIME_CRATE_ROOT}::generated::InterfaceKind::Stream, namespace: \"{}\", name: \"{}\", request_type: None, response_type: None, input_type: None, output_type: None, result_type: None, payload_type: Some(\"{}\"), stream_direction: Some(\"{}\"), annotations: \"{}\" }};\n",
                    namespace, s.name, field.type_ref, direction, ann
                ));
                out.push_str(&format!("{base_indent}}}\n"));
            }
            Interface::Command(c) => {
                out.push_str(&format!("{base_indent}pub mod {}_command {{\n", c.name));
                if let Some(field) = &c.input {
                    out.push_str(&format!(
                        "{base_indent}    pub type Goal = {};\n",
                        ridl_rust_type(&field.type_ref)?
                    ));
                }
                if let Some(field) = &c.output {
                    out.push_str(&format!(
                        "{base_indent}    pub type Feedback = {};\n",
                        ridl_rust_type(&field.type_ref)?
                    ));
                }
                if let Some(field) = &c.result {
                    out.push_str(&format!(
                        "{base_indent}    pub type Result = {};\n",
                        ridl_rust_type(&field.type_ref)?
                    ));
                }
                let ann = format_annotations_for_descriptor(&c.annotations);
                out.push_str(&format!(
                    "{base_indent}    pub const DESCRIPTOR: {RUNTIME_CRATE_ROOT}::generated::InterfaceDescriptor = {RUNTIME_CRATE_ROOT}::generated::InterfaceDescriptor {{ kind: {RUNTIME_CRATE_ROOT}::generated::InterfaceKind::Command, namespace: \"{}\", name: \"{}\", request_type: None, response_type: None, input_type: {}, output_type: {}, result_type: {}, payload_type: None, stream_direction: None, annotations: \"{}\" }};\n",
                    namespace,
                    c.name,
                    optional_str(c.input.as_ref().map(|f| f.type_ref.as_str())),
                    optional_str(c.output.as_ref().map(|f| f.type_ref.as_str())),
                    optional_str(c.result.as_ref().map(|f| f.type_ref.as_str())),
                    ann,
                ));
                out.push_str(&format!("{base_indent}}}\n"));
            }
        }
    }
    out.push_str("}\n\n");
    Ok(())
}

fn emit_query_module(out: &mut String, namespace: &str, q: &QueryDef, indent: &str) -> Result<()> {
    let ros_type = rosidl_symbol_name(namespace, &q.name);
    out.push_str(&format!(
        "{indent}pub const ROS_PACKAGE: &str = \"robonix_interfaces_ros2\";\n"
    ));
    out.push_str(&format!(
        "{indent}pub const ROS_SERVICE_TYPE: &str = \"{}\";\n",
        ros_type
    ));
    let ann = format_annotations_for_descriptor(&q.annotations);
    out.push_str(&format!(
        "{indent}pub const DESCRIPTOR: {RUNTIME_CRATE_ROOT}::generated::InterfaceDescriptor = {RUNTIME_CRATE_ROOT}::generated::InterfaceDescriptor {{ kind: {RUNTIME_CRATE_ROOT}::generated::InterfaceKind::Query, namespace: \"{}\", name: \"{}\", request_type: Some(\"{}\"), response_type: Some(\"{}\"), input_type: None, output_type: None, result_type: None, payload_type: None, stream_direction: None, annotations: \"{}\" }};\n",
        namespace, q.name, q.request.type_ref, q.response.type_ref, ann
    ));
    out.push_str(&format!(
        "{indent}pub async fn register(runtime_client: &mut {RUNTIME_CRATE_ROOT}::runtime::RuntimeClient, node_id: &str) -> anyhow::Result<std::string::String> {{\n"
    ));
    out.push_str(&format!(
        "{indent}    runtime_client.register_query(node_id, DESCRIPTOR.namespace, DESCRIPTOR.name).await\n"
    ));
    out.push_str(&format!("{indent}}}\n"));
    out.push_str(&format!(
        "{indent}pub async fn resolve(runtime_client: &mut {RUNTIME_CRATE_ROOT}::runtime::RuntimeClient, requester_id: &str, target: &str) -> anyhow::Result<std::string::String> {{\n"
    ));
    out.push_str(&format!(
        "{indent}    runtime_client.resolve_query(requester_id, target, DESCRIPTOR.namespace, DESCRIPTOR.name).await\n"
    ));
    out.push_str(&format!("{indent}}}\n"));

    if !is_string_query(q) {
        out.push_str(&format!(
            "{indent}// Transport wrappers are only generated for std_msgs/String query payloads in this PoC.\n"
        ));
        return Ok(());
    }

    let request_payload = ridl_rust_type(&q.request.type_ref)?;
    let response_payload = ridl_rust_type(&q.response.type_ref)?;
    let request_field = &q.request.name;
    let response_field = &q.response.name;

    out.push_str(&format!(
        "{indent}pub type RequestPayload = {};\n",
        request_payload
    ));
    out.push_str(&format!(
        "{indent}pub type ResponsePayload = {};\n",
        response_payload
    ));

    out.push_str(&format!(
        "{indent}#[derive(Clone, Debug, PartialEq, PartialOrd, serde::Serialize, serde::Deserialize)]\n"
    ));
    out.push_str(&format!("{indent}pub struct Request {{\n"));
    out.push_str(&format!(
        "{indent}    pub {}: RequestPayload,\n",
        request_field
    ));
    out.push_str(&format!("{indent}}}\n"));
    out.push_str(&format!("{indent}impl Default for Request {{\n"));
    out.push_str(&format!(
        "{indent}    fn default() -> Self {{ <Self as rosidl_runtime_rs::Message>::from_rmw_message(rmw::Request::default()) }}\n"
    ));
    out.push_str(&format!("{indent}}}\n"));
    out.push_str(&format!(
        "{indent}impl rosidl_runtime_rs::Message for Request {{\n"
    ));
    out.push_str(&format!("{indent}    type RmwMsg = rmw::Request;\n"));
    out.push_str(&format!(
        "{indent}    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {{\n"
    ));
    out.push_str(&format!("{indent}        match msg_cow {{\n"));
    out.push_str(&format!(
        "{indent}            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {{ {}: RequestPayload::into_rmw_message(std::borrow::Cow::Owned(msg.{})).into_owned() }}),\n",
        request_field, request_field
    ));
    out.push_str(&format!(
        "{indent}            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {{ {}: RequestPayload::into_rmw_message(std::borrow::Cow::Borrowed(&msg.{})).into_owned() }}),\n",
        request_field, request_field
    ));
    out.push_str(&format!("{indent}        }}\n"));
    out.push_str(&format!("{indent}    }}\n"));
    out.push_str(&format!(
        "{indent}    fn from_rmw_message(msg: Self::RmwMsg) -> Self {{ Self {{ {}: RequestPayload::from_rmw_message(msg.{}) }} }}\n",
        request_field, request_field
    ));
    out.push_str(&format!("{indent}}}\n"));

    out.push_str(&format!(
        "{indent}#[derive(Clone, Debug, PartialEq, PartialOrd, serde::Serialize, serde::Deserialize)]\n"
    ));
    out.push_str(&format!("{indent}pub struct Response {{\n"));
    out.push_str(&format!(
        "{indent}    pub {}: ResponsePayload,\n",
        response_field
    ));
    out.push_str(&format!("{indent}}}\n"));
    out.push_str(&format!("{indent}impl Default for Response {{\n"));
    out.push_str(&format!(
        "{indent}    fn default() -> Self {{ <Self as rosidl_runtime_rs::Message>::from_rmw_message(rmw::Response::default()) }}\n"
    ));
    out.push_str(&format!("{indent}}}\n"));
    out.push_str(&format!(
        "{indent}impl rosidl_runtime_rs::Message for Response {{\n"
    ));
    out.push_str(&format!("{indent}    type RmwMsg = rmw::Response;\n"));
    out.push_str(&format!(
        "{indent}    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {{\n"
    ));
    out.push_str(&format!("{indent}        match msg_cow {{\n"));
    out.push_str(&format!(
        "{indent}            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {{ {}: ResponsePayload::into_rmw_message(std::borrow::Cow::Owned(msg.{})).into_owned() }}),\n",
        response_field, response_field
    ));
    out.push_str(&format!(
        "{indent}            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {{ {}: ResponsePayload::into_rmw_message(std::borrow::Cow::Borrowed(&msg.{})).into_owned() }}),\n",
        response_field, response_field
    ));
    out.push_str(&format!("{indent}        }}\n"));
    out.push_str(&format!("{indent}    }}\n"));
    out.push_str(&format!(
        "{indent}    fn from_rmw_message(msg: Self::RmwMsg) -> Self {{ Self {{ {}: ResponsePayload::from_rmw_message(msg.{}) }} }}\n",
        response_field, response_field
    ));
    out.push_str(&format!("{indent}}}\n"));

    out.push_str(&format!("{indent}pub struct QueryService;\n"));
    out.push_str(&format!(
        "{indent}impl rosidl_runtime_rs::Service for QueryService {{\n"
    ));
    out.push_str(&format!("{indent}    type Request = Request;\n"));
    out.push_str(&format!("{indent}    type Response = Response;\n"));
    out.push_str(&format!(
        "{indent}    fn get_type_support() -> *const std::ffi::c_void {{\n"
    ));
    out.push_str(&format!(
        "{indent}        let func: unsafe extern \"C\" fn() -> *const std::ffi::c_void = {RUNTIME_CRATE_ROOT}::runtime::load_symbol(\"librobonix_interfaces_ros2__rosidl_typesupport_c.so\", b\"rosidl_typesupport_c__get_service_type_support_handle__robonix_interfaces_ros2__srv__{}\\0\");\n",
        ros_type
    ));
    out.push_str(&format!("{indent}        unsafe {{ func() }}\n"));
    out.push_str(&format!("{indent}    }}\n"));
    out.push_str(&format!("{indent}}}\n"));

    emit_query_rmw_module(out, indent, &ros_type, request_field, response_field);

    out.push_str(&format!(
        "{indent}pub fn request_from_string(value: impl Into<std::string::String>) -> Request {{\n"
    ));
    out.push_str(&format!(
        "{indent}    Request {{ {}: RequestPayload {{ data: value.into() }} }}\n",
        request_field
    ));
    out.push_str(&format!("{indent}}}\n"));
    out.push_str(&format!(
        "{indent}pub fn response_from_string(value: impl Into<std::string::String>) -> Response {{\n"
    ));
    out.push_str(&format!(
        "{indent}    Response {{ {}: ResponsePayload {{ data: value.into() }} }}\n",
        response_field
    ));
    out.push_str(&format!("{indent}}}\n"));
    out.push_str(&format!(
        "{indent}pub fn request_string(request: &Request) -> &str {{ request.{}.data.as_str() }}\n",
        request_field
    ));
    out.push_str(&format!(
        "{indent}pub fn response_into_string(response: Response) -> std::string::String {{ response.{}.data }}\n",
        response_field
    ));
    out.push_str(&format!(
        "{indent}pub fn create_server(node: &{RUNTIME_CRATE_ROOT}::runtime::RuntimeNode, channel_name: &str) -> anyhow::Result<{RUNTIME_CRATE_ROOT}::runtime::QueryServerHandle<QueryService>> {{\n"
    ));
    out.push_str(&format!(
        "{indent}    {RUNTIME_CRATE_ROOT}::runtime::create_query_server::<QueryService>(node, channel_name)\n"
    ));
    out.push_str(&format!("{indent}}}\n"));
    out.push_str(&format!(
        "{indent}pub fn create_client(node: &{RUNTIME_CRATE_ROOT}::runtime::RuntimeNode, channel_name: &str) -> anyhow::Result<{RUNTIME_CRATE_ROOT}::runtime::QueryClientHandle<QueryService>> {{\n"
    ));
    out.push_str(&format!(
        "{indent}    {RUNTIME_CRATE_ROOT}::runtime::create_query_client::<QueryService>(node, channel_name)\n"
    ));
    out.push_str(&format!("{indent}}}\n"));
    out.push_str(&format!(
        "{indent}pub async fn create_registered_server(node: &{RUNTIME_CRATE_ROOT}::runtime::RuntimeNode, runtime_client: &mut {RUNTIME_CRATE_ROOT}::runtime::RuntimeClient, node_id: &str) -> anyhow::Result<{RUNTIME_CRATE_ROOT}::runtime::QueryServerHandle<QueryService>> {{\n"
    ));
    out.push_str(&format!(
        "{indent}    let channel_name = register(runtime_client, node_id).await?;\n"
    ));
    out.push_str(&format!("{indent}    create_server(node, &channel_name)\n"));
    out.push_str(&format!("{indent}}}\n"));
    out.push_str(&format!(
        "{indent}pub async fn create_resolved_client(node: &{RUNTIME_CRATE_ROOT}::runtime::RuntimeNode, runtime_client: &mut {RUNTIME_CRATE_ROOT}::runtime::RuntimeClient, requester_id: &str, target: &str) -> anyhow::Result<{RUNTIME_CRATE_ROOT}::runtime::QueryClientHandle<QueryService>> {{\n"
    ));
    out.push_str(&format!(
        "{indent}    let channel_name = resolve(runtime_client, requester_id, target).await?;\n"
    ));
    out.push_str(&format!("{indent}    create_client(node, &channel_name)\n"));
    out.push_str(&format!("{indent}}}\n"));
    Ok(())
}

fn emit_query_rmw_module(
    out: &mut String,
    indent: &str,
    ros_type: &str,
    request_field: &str,
    response_field: &str,
) {
    out.push_str(&format!("{indent}pub mod rmw {{\n"));

    out.push_str(&format!("{indent}    #[repr(C)]\n"));
    out.push_str(&format!(
        "{indent}    #[derive(Clone, Debug, PartialEq, PartialOrd)]\n"
    ));
    out.push_str(&format!("{indent}    pub struct Request {{\n"));
    out.push_str(&format!(
        "{indent}        pub {}: {RUNTIME_CRATE_ROOT}::generated::types::std_msgs::rmw::String,\n",
        request_field
    ));
    out.push_str(&format!("{indent}    }}\n"));
    out.push_str(&format!("{indent}    impl Default for Request {{\n"));
    out.push_str(&format!("{indent}        fn default() -> Self {{\n"));
    out.push_str(&format!("{indent}            unsafe {{\n"));
    out.push_str(&format!(
        "{indent}                let mut msg = std::mem::zeroed();\n"
    ));
    out.push_str(&format!(
        "{indent}                let init: unsafe extern \"C\" fn(*mut Request) -> bool = {RUNTIME_CRATE_ROOT}::runtime::load_symbol(\"librobonix_interfaces_ros2__rosidl_generator_c.so\", b\"robonix_interfaces_ros2__srv__{}_Request__init\\0\");\n",
        ros_type
    ));
    out.push_str(&format!(
        "{indent}                if !init(&mut msg as *mut _) {{ panic!(\"Call to robonix_interfaces_ros2__srv__{}_Request__init() failed\"); }}\n",
        ros_type
    ));
    out.push_str(&format!("{indent}                msg\n"));
    out.push_str(&format!("{indent}            }}\n"));
    out.push_str(&format!("{indent}        }}\n"));
    out.push_str(&format!("{indent}    }}\n"));
    out.push_str(&format!(
        "{indent}    impl rosidl_runtime_rs::Message for Request {{\n"
    ));
    out.push_str(&format!("{indent}        type RmwMsg = Self;\n"));
    out.push_str(&format!(
        "{indent}        fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {{ msg_cow }}\n"
    ));
    out.push_str(&format!(
        "{indent}        fn from_rmw_message(msg: Self::RmwMsg) -> Self {{ msg }}\n"
    ));
    out.push_str(&format!("{indent}    }}\n"));
    out.push_str(&format!(
        "{indent}    impl rosidl_runtime_rs::RmwMessage for Request {{\n"
    ));
    out.push_str(&format!(
        "{indent}        const TYPE_NAME: &'static str = \"robonix_interfaces_ros2/srv/{}_Request\";\n",
        ros_type
    ));
    out.push_str(&format!(
        "{indent}        fn get_type_support() -> *const std::ffi::c_void {{\n"
    ));
    out.push_str(&format!(
        "{indent}            let func: unsafe extern \"C\" fn() -> *const std::ffi::c_void = {RUNTIME_CRATE_ROOT}::runtime::load_symbol(\"librobonix_interfaces_ros2__rosidl_typesupport_c.so\", b\"rosidl_typesupport_c__get_message_type_support_handle__robonix_interfaces_ros2__srv__{}_Request\\0\");\n",
        ros_type
    ));
    out.push_str(&format!("{indent}            unsafe {{ func() }}\n"));
    out.push_str(&format!("{indent}        }}\n"));
    out.push_str(&format!("{indent}    }}\n"));

    out.push_str(&format!("{indent}    #[repr(C)]\n"));
    out.push_str(&format!(
        "{indent}    #[derive(Clone, Debug, PartialEq, PartialOrd)]\n"
    ));
    out.push_str(&format!("{indent}    pub struct Response {{\n"));
    out.push_str(&format!(
        "{indent}        pub {}: {RUNTIME_CRATE_ROOT}::generated::types::std_msgs::rmw::String,\n",
        response_field
    ));
    out.push_str(&format!("{indent}    }}\n"));
    out.push_str(&format!("{indent}    impl Default for Response {{\n"));
    out.push_str(&format!("{indent}        fn default() -> Self {{\n"));
    out.push_str(&format!("{indent}            unsafe {{\n"));
    out.push_str(&format!(
        "{indent}                let mut msg = std::mem::zeroed();\n"
    ));
    out.push_str(&format!(
        "{indent}                let init: unsafe extern \"C\" fn(*mut Response) -> bool = {RUNTIME_CRATE_ROOT}::runtime::load_symbol(\"librobonix_interfaces_ros2__rosidl_generator_c.so\", b\"robonix_interfaces_ros2__srv__{}_Response__init\\0\");\n",
        ros_type
    ));
    out.push_str(&format!(
        "{indent}                if !init(&mut msg as *mut _) {{ panic!(\"Call to robonix_interfaces_ros2__srv__{}_Response__init() failed\"); }}\n",
        ros_type
    ));
    out.push_str(&format!("{indent}                msg\n"));
    out.push_str(&format!("{indent}            }}\n"));
    out.push_str(&format!("{indent}        }}\n"));
    out.push_str(&format!("{indent}    }}\n"));
    out.push_str(&format!(
        "{indent}    impl rosidl_runtime_rs::Message for Response {{\n"
    ));
    out.push_str(&format!("{indent}        type RmwMsg = Self;\n"));
    out.push_str(&format!(
        "{indent}        fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {{ msg_cow }}\n"
    ));
    out.push_str(&format!(
        "{indent}        fn from_rmw_message(msg: Self::RmwMsg) -> Self {{ msg }}\n"
    ));
    out.push_str(&format!("{indent}    }}\n"));
    out.push_str(&format!(
        "{indent}    impl rosidl_runtime_rs::RmwMessage for Response {{\n"
    ));
    out.push_str(&format!(
        "{indent}        const TYPE_NAME: &'static str = \"robonix_interfaces_ros2/srv/{}_Response\";\n",
        ros_type
    ));
    out.push_str(&format!(
        "{indent}        fn get_type_support() -> *const std::ffi::c_void {{\n"
    ));
    out.push_str(&format!(
        "{indent}            let func: unsafe extern \"C\" fn() -> *const std::ffi::c_void = {RUNTIME_CRATE_ROOT}::runtime::load_symbol(\"librobonix_interfaces_ros2__rosidl_typesupport_c.so\", b\"rosidl_typesupport_c__get_message_type_support_handle__robonix_interfaces_ros2__srv__{}_Response\\0\");\n",
        ros_type
    ));
    out.push_str(&format!("{indent}            unsafe {{ func() }}\n"));
    out.push_str(&format!("{indent}        }}\n"));
    out.push_str(&format!("{indent}    }}\n"));
    out.push_str(&format!("{indent}}}\n"));
}

fn is_string_query(q: &QueryDef) -> bool {
    q.request.type_ref == "std_msgs/msg/String" && q.response.type_ref == "std_msgs/msg/String"
}

fn ridl_rust_type(type_ref: &str) -> Result<String> {
    let Some((package, name)) = parse_ridl_type_ref(type_ref) else {
        bail!(
            "{RIDLC_ERR_PREFIX} unsupported RIDL type '{}' (expected format: package/msg/Name, e.g. std_msgs/msg/String)",
            type_ref
        );
    };
    Ok(format!(
        "{RUNTIME_CRATE_ROOT}::generated::types::{}::{}",
        package, name
    ))
}
