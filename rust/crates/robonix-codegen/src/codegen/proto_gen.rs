// SPDX-License-Identifier: MulanPSL-2.0
// Protobuf (.proto) code generator — ROS IDL -> proto3
//
// Emits **one `{package}.proto` per ROS package** that has content:
//   - Every `.msg` -> a `message` (always).
//   - `*_Request` / `*_Response` from `.srv` **only** when `--contracts` is passed and that srv is listed in a contract `[io.srv].srv`.
// Does **not** emit per-package `service FooService { rpc ... }`; gRPC facades are **only** in `robonix_contracts.proto`.

use anyhow::{Context, Result};
use std::collections::BTreeSet;
use std::fmt::Write as FmtWrite;
use std::fs;
use std::path::Path;

use super::msg_parser::{MsgField, MsgResolver, MsgSpec, MsgTypeRef, RosPrimitive, SrvSpec};

fn proto_primitive_type(p: &str) -> &'static str {
    // Single source of truth for primitive → proto3 mapping is
    // RosPrimitive::proto_type. Panic on a primitive the parser
    // wouldn't have accepted — that's a codegen bug, not a runtime
    // condition we want to silently swallow as `bytes` (which used to
    // be the catch-all and produced wrong wire formats).
    let prim = RosPrimitive::parse(p).unwrap_or_else(|| {
        panic!(
            "[robonix-codegen] proto: primitive '{p}' is not in RosPrimitive::parse — \
             every primitive accepted by the parser must have a proto mapping; \
             extend RosPrimitive if this is a new ROS primitive."
        )
    });
    prim.proto_type()
}

fn proto_field_type(field: &MsgField, current_package: &str) -> String {
    let base = match &field.type_ref {
        MsgTypeRef::Primitive(p) => {
            // `uint8[]` (unsigned 8-bit, unbounded) is the canonical
            // raw-byte buffer and becomes proto `bytes`. `uint8[N]`
            // (fixed-size) does NOT collapse to `bytes` because proto3
            // bytes has no length constraint, so a fixed-size array
            // would lose its size invariant on the wire — keep it as
            // `repeated uint32` and let the consumer enforce length.
            // `byte[]` is signed 8-bit and stays `repeated int32`.
            let prim = RosPrimitive::parse(p).unwrap_or_else(|| {
                panic!("[robonix-codegen] proto: primitive '{p}' is not in RosPrimitive::parse")
            });
            if field.is_array && field.array_size.is_none() && prim.is_blob_element() {
                return "bytes".to_string();
            }
            proto_primitive_type(p).to_string()
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

fn import_named_type(imports: &mut BTreeSet<String>, current_package: &str, tr: &MsgTypeRef) {
    if let MsgTypeRef::Named { package, .. } = tr
        && package != current_package
    {
        imports.insert(package.clone());
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
        for field in srv.request.fields.iter().chain(srv.response.fields.iter()) {
            import_named_type(&mut imports, current_package, &field.type_ref);
        }
    }
    imports
}

pub fn generate(
    resolver: &MsgResolver,
    out_dir: &Path,
    contract_srvs: Option<&BTreeSet<(String, String)>>,
    verbose: bool,
) -> Result<()> {
    fs::create_dir_all(out_dir)?;

    let mut all_packages = BTreeSet::new();
    for spec in resolver.ordered_specs() {
        all_packages.insert(spec.package.clone());
    }
    if let Some(set) = contract_srvs {
        for (pkg, _) in set.iter() {
            all_packages.insert(pkg.clone());
        }
    }

    for package in &all_packages {
        let specs: Vec<_> = resolver
            .ordered_specs()
            .into_iter()
            .filter(|s| &s.package == package)
            .collect();
        let srvs: Vec<_> = match contract_srvs {
            Some(set) => resolver
                .ordered_srvs()
                .into_iter()
                .filter(|s| {
                    &s.package == package && set.contains(&(s.package.clone(), s.name.clone()))
                })
                .collect(),
            None => Vec::new(),
        };

        if specs.is_empty() && srvs.is_empty() {
            continue;
        }

        let mut out = String::new();
        let _ = writeln!(out, "// @generated by robonix-codegen --lang proto");
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

        for spec in &specs {
            emit_message(&mut out, spec);
            let _ = writeln!(out);
        }

        for srv in &srvs {
            emit_srv_messages(&mut out, srv);
            let _ = writeln!(out);
        }

        let filename = format!("{}.proto", package);
        let filepath = out_dir.join(&filename);
        fs::write(&filepath, &out)
            .with_context(|| format!("failed to write proto file to '{}'", filepath.display()))?;
        if verbose {
            eprintln!(
                "[robonix-codegen] generated proto for '{}' ({} msgs, {} contract srvs) -> {}",
                package,
                specs.len(),
                srvs.len(),
                filepath.display()
            );
        }
    }

    Ok(())
}
