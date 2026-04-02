// SPDX-License-Identifier: MulanPSL-2.0
// Protobuf (.proto) code generator — ROS .msg/.srv -> proto3
//
// Generates one .proto file per ROS package, containing:
//   - A `message` block for each .msg type
//   - A `service` block grouping all RPCs derived from .srv files

use anyhow::{Context, Result};
use std::collections::BTreeSet;
use std::fmt::Write as FmtWrite;
use std::fs;
use std::path::Path;

use super::msg_parser::{GrpcStreamMode, MsgField, MsgResolver, MsgSpec, MsgTypeRef, SrvSpec};

fn ros_to_proto_primitive(ros_type: &str) -> &'static str {
    match ros_type {
        "bool" => "bool",
        "byte" | "uint8" => "uint32",
        "char" | "int8" => "int32",
        "uint16" => "uint32",
        "int16" => "int32",
        "uint32" => "uint32",
        "int32" => "int32",
        "uint64" => "uint64",
        "int64" => "int64",
        "float32" => "float",
        "float64" => "double",
        "string" | "wstring" => "string",
        _ => "bytes",
    }
}

fn proto_field_type(field: &MsgField, current_package: &str) -> String {
    let base = match &field.type_ref {
        MsgTypeRef::Primitive(p) => {
            if field.is_array && (p == "uint8" || p == "byte") {
                return "bytes".to_string();
            }
            ros_to_proto_primitive(p).to_string()
        }
        MsgTypeRef::Named { package, name } => {
            if package == current_package {
                name.clone()
            } else {
                format!("{}.{}", proto_package_name(package), name)
            }
        }
    };
    if field.is_array {
        format!("repeated {}", base)
    } else {
        base
    }
}

/// ROS package name (`prm_base`, `sensor_msgs`) → protobuf package (`robonix.prm_base`, …).
pub fn proto_package_name(ros_package: &str) -> String {
    format!("robonix.{}", ros_package)
}

fn emit_message(out: &mut String, spec: &MsgSpec) {
    let _ = writeln!(out, "message {} {{", spec.name);
    for (i, field) in spec.fields.iter().enumerate() {
        let proto_type = proto_field_type(field, &spec.package);
        let _ = writeln!(out, "  {} {} = {};", proto_type, field.name, i + 1);
    }
    let _ = writeln!(out, "}}");
}

fn emit_srv_messages(out: &mut String, srv: &SrvSpec) {
    emit_message(out, &srv.request);
    let _ = writeln!(out);
    emit_message(out, &srv.response);
}

fn stream_element_proto_type(element: &MsgTypeRef, current_package: &str) -> String {
    match element {
        MsgTypeRef::Primitive(p) => ros_to_proto_primitive(p).to_string(),
        MsgTypeRef::Named { package, name } => {
            if package == current_package {
                name.clone()
            } else {
                format!("{}.{}", proto_package_name(package), name)
            }
        }
    }
}

fn emit_service(out: &mut String, package: &str, srvs: &[&SrvSpec]) {
    let service_name = format!("{}Service", to_pascal_case(package));
    let _ = writeln!(out, "service {} {{", service_name);
    for srv in srvs {
        match &srv.grpc_stream {
            Some(GrpcStreamMode::ServerStream { element }) => {
                let el = stream_element_proto_type(element, package);
                let _ = writeln!(
                    out,
                    "  rpc {} ({}) returns (stream {});",
                    srv.name, srv.request.name, el
                );
            }
            Some(GrpcStreamMode::ClientStream { element }) => {
                let el = stream_element_proto_type(element, package);
                let _ = writeln!(
                    out,
                    "  rpc {} (stream {}) returns ({});",
                    srv.name, el, srv.response.name
                );
            }
            None => {
                let _ = writeln!(
                    out,
                    "  rpc {} ({}) returns ({});",
                    srv.name, srv.request.name, srv.response.name,
                );
            }
        }
    }
    let _ = writeln!(out, "}}");
}

fn to_pascal_case(s: &str) -> String {
    s.split('_')
        .filter(|p| !p.is_empty())
        .map(|p| {
            let mut c = p.chars();
            match c.next() {
                None => String::new(),
                Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
            }
        })
        .collect()
}

fn import_named_type(imports: &mut BTreeSet<String>, current_package: &str, tr: &MsgTypeRef) {
    if let MsgTypeRef::Named { package, .. } = tr {
        if package != current_package {
            imports.insert(package.clone());
        }
    }
}

fn collect_imports(
    specs: &[&MsgSpec],
    srvs: &[&SrvSpec],
    current_package: &str,
) -> BTreeSet<String> {
    let mut imports = BTreeSet::new();
    for spec in specs {
        for field in &spec.fields {
            import_named_type(&mut imports, current_package, &field.type_ref);
        }
    }
    for srv in srvs {
        if let Some(mode) = &srv.grpc_stream {
            let element = match mode {
                GrpcStreamMode::ServerStream { element }
                | GrpcStreamMode::ClientStream { element } => element,
            };
            import_named_type(&mut imports, current_package, element);
        }
        for field in srv.request.fields.iter().chain(srv.response.fields.iter()) {
            import_named_type(&mut imports, current_package, &field.type_ref);
        }
    }
    imports
}

pub fn generate(resolver: &MsgResolver, out_dir: &Path) -> Result<()> {
    fs::create_dir_all(out_dir)?;

    let mut all_packages = BTreeSet::new();
    for spec in resolver.ordered_specs() {
        all_packages.insert(spec.package.clone());
    }
    for srv in resolver.ordered_srvs() {
        all_packages.insert(srv.package.clone());
    }

    for package in &all_packages {
        let specs: Vec<_> = resolver
            .ordered_specs()
            .into_iter()
            .filter(|s| &s.package == package)
            .collect();
        let srvs: Vec<_> = resolver
            .ordered_srvs()
            .into_iter()
            .filter(|s| &s.package == package)
            .collect();

        if specs.is_empty() && srvs.is_empty() {
            continue;
        }

        let mut out = String::new();
        let _ = writeln!(out, "// @generated by ridlc --lang proto");
        let _ = writeln!(out, "// source: ROS IDL package '{}'", package);
        let _ = writeln!(out, "syntax = \"proto3\";");
        let _ = writeln!(out);
        let _ = writeln!(out, "package {};", proto_package_name(package));
        let _ = writeln!(out);

        let imports = collect_imports(&specs, &srvs, package);
        for imp in &imports {
            let _ = writeln!(out, "import \"{}.proto\";", imp);
        }
        if !imports.is_empty() {
            let _ = writeln!(out);
        }

        // Emit all message definitions
        for spec in &specs {
            emit_message(&mut out, spec);
            let _ = writeln!(out);
        }

        // Emit request/response messages for services, then the service block
        if !srvs.is_empty() {
            for srv in &srvs {
                emit_srv_messages(&mut out, srv);
                let _ = writeln!(out);
            }
            emit_service(&mut out, package, &srvs);
            let _ = writeln!(out);
        }

        let filename = format!("{}.proto", package);
        let filepath = out_dir.join(&filename);
        fs::write(&filepath, &out)
            .with_context(|| format!("failed to write proto file to '{}'", filepath.display()))?;
        eprintln!(
            "[ridlc] generated proto for '{}' ({} msgs, {} srvs) -> {}",
            package,
            specs.len(),
            srvs.len(),
            filepath.display()
        );
    }

    Ok(())
}
