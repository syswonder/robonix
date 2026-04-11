// SPDX-License-Identifier: MulanPSL-2.0
// mcp_python_gen.rs — Python @dataclass generator for MCP tool use (--lang mcp)
//
// Generates one `{package}_mcp.py` per ROS package, containing:
//   - @dataclass for each .msg type
//   - to_dict() / from_dict() for JSON serialization over MCP wire
//   - json_schema() classmethod for MCP tool registration (input_schema / output_schema)
//
// Unlike --lang python (ctypes structs for iceoryx2), nested types are NOT flattened;
// they appear as nested objects in both Python and JSON Schema.
// uint8[] binary blobs are represented as `bytes` and base64-encoded in JSON.

use anyhow::{Context, Result};
use std::collections::BTreeSet;
use std::fmt::Write as FmtWrite;
use std::fs;
use std::path::Path;

use super::msg_parser::{MsgField, MsgResolver, MsgSpec, MsgTypeRef};

// ── Primitive type mappings ──────────────────────────────────────────────────

fn ros_to_python_type(t: &str, is_array: bool, is_uint8_blob: bool) -> String {
    if is_uint8_blob {
        return if is_array {
            "bytes".to_string()
        } else {
            "int".to_string()
        };
    }
    let base = match t {
        "bool" => "bool",
        "float32" | "float64" => "float",
        "string" | "wstring" => "str",
        _ => "int",
    };
    if is_array {
        format!("List[{}]", base)
    } else {
        base.to_string()
    }
}

fn ros_to_python_default(t: &str, is_array: bool, is_uint8_blob: bool) -> String {
    if is_uint8_blob {
        return "b\"\"".to_string();
    }
    if is_array {
        return "field(default_factory=list)".to_string();
    }
    match t {
        "bool" => "False".to_string(),
        "float32" | "float64" => "0.0".to_string(),
        "string" | "wstring" => "\"\"".to_string(),
        _ => "0".to_string(),
    }
}

fn ros_to_json_schema_type(t: &str) -> &'static str {
    match t {
        "bool" => "boolean",
        "float32" | "float64" => "number",
        "string" | "wstring" => "string",
        _ => "integer",
    }
}

// ── Cross-package dependency collection ──────────────────────────────────────

fn cross_package_deps(spec: &MsgSpec) -> BTreeSet<String> {
    let mut pkgs = BTreeSet::new();
    for f in &spec.fields {
        if let MsgTypeRef::Named { package, .. } = &f.type_ref
            && package != &spec.package
        {
            pkgs.insert(package.clone());
        }
    }
    pkgs
}

// ── Field code emitters ───────────────────────────────────────────────────────

/// Returns true if this field is the `uint8[]` blob pattern (raw binary data).
fn is_uint8_blob(field: &MsgField) -> bool {
    field.is_array
        && field.array_size.is_none()
        && matches!(&field.type_ref, MsgTypeRef::Primitive(p) if p == "uint8" || p == "byte")
}

fn emit_field_decl(out: &mut String, field: &MsgField, pkg: &str) {
    let type_str = match &field.type_ref {
        MsgTypeRef::Primitive(p) => {
            let blob = is_uint8_blob(field);
            ros_to_python_type(p, field.is_array && !blob, blob)
        }
        MsgTypeRef::Named { package, name } => {
            let qualified = if package == pkg {
                name.clone()
            } else {
                format!("{}_mcp.{}", package, name)
            };
            if field.is_array {
                format!("List[{}]", qualified)
            } else {
                qualified
            }
        }
    };

    let default_str = match &field.type_ref {
        MsgTypeRef::Primitive(p) => {
            let blob = is_uint8_blob(field);
            ros_to_python_default(p, field.is_array && !blob, blob)
        }
        MsgTypeRef::Named { package, name } => {
            if field.is_array {
                "field(default_factory=list)".to_string()
            } else {
                let qualified = if package == pkg {
                    name.clone()
                } else {
                    format!("{}_mcp.{}", package, name)
                };
                // Defer constructor to runtime: same-package msgs are emitted in
                // arbitrary order; `default_factory=Foo` evaluates `Foo` at class body time.
                format!("field(default_factory=lambda: {}())", qualified)
            }
        }
    };

    let _ = writeln!(out, "    {}: {} = {}", field.name, type_str, default_str);
}

fn emit_to_dict_entry(out: &mut String, field: &MsgField) {
    let blob = is_uint8_blob(field);
    match &field.type_ref {
        MsgTypeRef::Primitive(_) if blob => {
            let _ = writeln!(
                out,
                "            \"{name}\": _b64enc(self.{name}),",
                name = field.name
            );
        }
        MsgTypeRef::Primitive(_) => {
            let _ = writeln!(
                out,
                "            \"{name}\": self.{name},",
                name = field.name
            );
        }
        MsgTypeRef::Named { .. } => {
            if field.is_array {
                let _ = writeln!(
                    out,
                    "            \"{name}\": [_i.to_dict() for _i in self.{name}],",
                    name = field.name
                );
            } else {
                let _ = writeln!(
                    out,
                    "            \"{name}\": self.{name}.to_dict(),",
                    name = field.name
                );
            }
        }
    }
}

fn emit_from_dict_entry(out: &mut String, field: &MsgField, pkg: &str) {
    let blob = is_uint8_blob(field);
    match &field.type_ref {
        MsgTypeRef::Primitive(_) if blob => {
            let _ = writeln!(
                out,
                "            {name}=_b64dec(d.get(\"{name}\", \"\")),",
                name = field.name
            );
        }
        MsgTypeRef::Primitive(p) => {
            let cast = match p.as_str() {
                "bool" => "bool",
                "float32" | "float64" => "float",
                "string" | "wstring" => "str",
                _ => "int",
            };
            let default = ros_to_python_default(p, field.is_array, false);
            if field.is_array {
                let _ = writeln!(
                    out,
                    "            {name}=[{cast}(_x) for _x in d.get(\"{name}\", [])],",
                    name = field.name,
                    cast = cast
                );
            } else {
                let _ = writeln!(
                    out,
                    "            {name}={cast}(d.get(\"{name}\", {default})),",
                    name = field.name,
                    cast = cast,
                    default = default
                );
            }
        }
        MsgTypeRef::Named {
            package,
            name: type_name,
        } => {
            let qualified = if package == pkg {
                type_name.clone()
            } else {
                format!("{}_mcp.{}", package, type_name)
            };
            if field.is_array {
                let _ = writeln!(
                    out,
                    "            {name}=[{qual}.from_dict(_x) for _x in d.get(\"{name}\", [])],",
                    name = field.name,
                    qual = qualified
                );
            } else {
                let _ = writeln!(
                    out,
                    "            {name}={qual}.from_dict(d.get(\"{name}\", {{}})),",
                    name = field.name,
                    qual = qualified
                );
            }
        }
    }
}

fn emit_json_schema_prop(out: &mut String, field: &MsgField, pkg: &str, depth: usize) {
    let indent = "    ".repeat(depth + 3);
    let blob = is_uint8_blob(field);

    match &field.type_ref {
        MsgTypeRef::Primitive(_) if blob => {
            let _ = writeln!(
                out,
                "{indent}\"{name}\": {{\"type\": \"string\", \"contentEncoding\": \"base64\"}},",
                indent = indent,
                name = field.name
            );
        }
        MsgTypeRef::Primitive(p) => {
            let json_t = ros_to_json_schema_type(p);
            if field.is_array {
                let _ = writeln!(
                    out,
                    "{indent}\"{name}\": {{\"type\": \"array\", \"items\": {{\"type\": \"{json_t}\"}}}},",
                    indent = indent,
                    name = field.name,
                    json_t = json_t
                );
            } else {
                let _ = writeln!(
                    out,
                    "{indent}\"{name}\": {{\"type\": \"{json_t}\"}},",
                    indent = indent,
                    name = field.name,
                    json_t = json_t
                );
            }
        }
        MsgTypeRef::Named {
            package,
            name: type_name,
        } => {
            let qualified = if package == pkg {
                format!("{}.json_schema()", type_name)
            } else {
                format!("{}_mcp.{}.json_schema()", package, type_name)
            };
            if field.is_array {
                let _ = writeln!(
                    out,
                    "{indent}\"{name}\": {{\"type\": \"array\", \"items\": {qual}}},",
                    indent = indent,
                    name = field.name,
                    qual = qualified
                );
            } else {
                let _ = writeln!(
                    out,
                    "{indent}\"{name}\": {qual},",
                    indent = indent,
                    name = field.name,
                    qual = qualified
                );
            }
        }
    }
}

// ── Class emitter ─────────────────────────────────────────────────────────────

fn emit_class(out: &mut String, spec: &MsgSpec) {
    let _ = writeln!(out, "@dataclass");
    let _ = writeln!(out, "class {}:", spec.name);
    let _ = writeln!(
        out,
        "    \"\"\"ROS IDL: {}/msg/{}\"\"\"",
        spec.package, spec.name
    );

    if spec.fields.is_empty() {
        let _ = writeln!(out, "    _empty: bool = field(default=True, repr=False)");
        let _ = writeln!(out);
        let _ = writeln!(out, "    def to_dict(self) -> dict:");
        let _ = writeln!(out, "        return {{}}");
        let _ = writeln!(out);
        let _ = writeln!(out, "    @classmethod");
        let _ = writeln!(
            out,
            "    def from_dict(cls, _d: dict) -> \"{}\":",
            spec.name
        );
        let _ = writeln!(out, "        return cls()");
        let _ = writeln!(out);
        let _ = writeln!(out, "    @classmethod");
        let _ = writeln!(out, "    def json_schema(cls) -> dict:");
        let _ = writeln!(
            out,
            "        return {{\"type\": \"object\", \"properties\": {{}}}}"
        );
        return;
    }

    // ── Field declarations ──────────────────────────────────────────────────
    for f in &spec.fields {
        emit_field_decl(out, f, &spec.package);
    }

    // ── to_dict() ───────────────────────────────────────────────────────────
    let _ = writeln!(out);
    let _ = writeln!(out, "    def to_dict(self) -> dict:");
    let _ = writeln!(out, "        return {{");
    for f in &spec.fields {
        emit_to_dict_entry(out, f);
    }
    let _ = writeln!(out, "        }}");

    // ── from_dict() ─────────────────────────────────────────────────────────
    let _ = writeln!(out);
    let _ = writeln!(out, "    @classmethod");
    let _ = writeln!(out, "    def from_dict(cls, d: dict) -> \"{}\":", spec.name);
    let _ = writeln!(out, "        return cls(");
    for f in &spec.fields {
        emit_from_dict_entry(out, f, &spec.package);
    }
    let _ = writeln!(out, "        )");

    // ── json_schema() ────────────────────────────────────────────────────────
    let _ = writeln!(out);
    let _ = writeln!(out, "    @classmethod");
    let _ = writeln!(out, "    def json_schema(cls) -> dict:");
    let _ = writeln!(out, "        return {{");
    let _ = writeln!(out, "            \"type\": \"object\",");
    let _ = writeln!(out, "            \"properties\": {{");
    for f in &spec.fields {
        emit_json_schema_prop(out, f, &spec.package, 0);
    }
    let _ = writeln!(out, "            }},");
    let _ = writeln!(out, "        }}");
}

// ── Public entry point ────────────────────────────────────────────────────────

pub fn generate(resolver: &MsgResolver, out_dir: &Path, verbose: bool) -> Result<()> {
    fs::create_dir_all(out_dir)?;

    let mut all_packages: BTreeSet<String> = BTreeSet::new();
    for spec in resolver.ordered_specs() {
        all_packages.insert(spec.package.clone());
    }

    for package in &all_packages {
        let specs: Vec<_> = resolver
            .ordered_specs()
            .into_iter()
            .filter(|s| &s.package == package)
            .collect();

        if specs.is_empty() {
            continue;
        }

        // Collect cross-package imports needed by any spec in this package
        let mut ext_pkgs: BTreeSet<String> = BTreeSet::new();
        for spec in &specs {
            for dep in cross_package_deps(spec) {
                if &dep != package {
                    ext_pkgs.insert(dep);
                }
            }
        }

        let mut out = String::new();
        let _ = writeln!(
            out,
            "# @generated by robonix-codegen --lang mcp — DO NOT EDIT"
        );
        let _ = writeln!(out, "# source: ROS IDL package '{}'", package);
        let _ = writeln!(
            out,
            "# Python dataclasses for MCP tool use (to_dict / from_dict / json_schema)."
        );
        let _ = writeln!(
            out,
            "# Re-generate: cargo run -p robonix-codegen -- --lang mcp -I <lib> -o <out>"
        );
        let _ = writeln!(out);
        let _ = writeln!(out, "from __future__ import annotations");
        let _ = writeln!(out, "import base64");
        let _ = writeln!(out, "from dataclasses import dataclass, field");
        let _ = writeln!(out, "from typing import List");
        for dep in &ext_pkgs {
            let _ = writeln!(out, "import {}_mcp", dep);
        }
        let _ = writeln!(out);
        let _ = writeln!(out, "def _b64enc(b: bytes) -> str:");
        let _ = writeln!(out, "    return base64.b64encode(b).decode(\"ascii\")");
        let _ = writeln!(out);
        let _ = writeln!(out, "def _b64dec(s: str) -> bytes:");
        let _ = writeln!(out, "    return base64.b64decode(s) if s else b\"\"");
        let _ = writeln!(out);
        let _ = writeln!(out);

        for spec in &specs {
            emit_class(&mut out, spec);
            let _ = writeln!(out);
            let _ = writeln!(out);
        }

        let filename = format!("{}_mcp.py", package);
        let filepath = out_dir.join(&filename);
        fs::write(&filepath, &out)
            .with_context(|| format!("failed to write '{}'", filepath.display()))?;
        if verbose {
            eprintln!(
                "[robonix-codegen] mcp: '{}' ({} msgs) -> {}",
                package,
                specs.len(),
                filepath.display()
            );
        }
    }

    // Package __init__.py
    let init = out_dir.join("__init__.py");
    if !init.exists() {
        fs::write(&init, "# @generated by robonix-codegen --lang mcp\n")?;
    }

    Ok(())
}
