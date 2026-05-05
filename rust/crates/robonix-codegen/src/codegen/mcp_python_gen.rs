// SPDX-License-Identifier: MulanPSL-2.0
// mcp_python_gen.rs — Python @dataclass generator for MCP tool use (--lang mcp)
//
// Generates one `{package}_mcp.py` per ROS package, containing:
//   - @dataclass for each .msg type
//   - to_dict() / from_dict() for JSON serialization over MCP wire
//   - json_schema() classmethod for MCP tool registration (input_schema / output_schema)
//
// Nested types appear as nested objects in both Python and JSON Schema
// (no flattening). `uint8[]` binary blobs are represented as `bytes` and
// base64-encoded in the JSON form.

use anyhow::{Context, Result};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as FmtWrite;
use std::fs;
use std::path::Path;

use super::msg_parser::{MsgField, MsgResolver, MsgSpec, MsgTypeRef, RosPrimitive};

// ── Primitive type mappings ──────────────────────────────────────────────────
//
// Every primitive flows through `RosPrimitive`, the codegen's single
// source of truth. Unknown primitives panic at codegen time rather
// than silently turning into `int` (the previous catch-all behaviour
// would coerce e.g. a typo'd `uint128` into `int` on the wire).

fn parse_prim_or_panic(p: &str) -> RosPrimitive {
    RosPrimitive::parse(p).unwrap_or_else(|| {
        panic!(
            "[robonix-codegen] mcp: primitive '{p}' is not in RosPrimitive::parse — \
             every primitive accepted by the parser must have an MCP mapping."
        )
    })
}

fn python_field_type(p: &str, is_array: bool, is_blob: bool) -> String {
    if is_blob {
        // Both `uint8[]` and `uint8[N]` are bytes; the fixed-size
        // case carries its length in JSON Schema (handled later) but
        // the in-memory type is the same.
        return "bytes".to_string();
    }
    let base = parse_prim_or_panic(p).python_type();
    if is_array {
        format!("List[{}]", base)
    } else {
        base.to_string()
    }
}

fn python_default(p: &str, is_array: bool, is_blob: bool) -> String {
    if is_blob {
        return "b\"\"".to_string();
    }
    if is_array {
        return "field(default_factory=list)".to_string();
    }
    parse_prim_or_panic(p).python_default().to_string()
}

fn python_cast(p: &str) -> &'static str {
    parse_prim_or_panic(p).python_cast()
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

/// Returns true when this field's wire shape is "raw byte buffer".
/// Only `uint8[]` and `uint8[N]` qualify — `byte` is signed 8-bit and
/// must NOT be conflated with `uint8` (unsigned) on the wire. Both
/// the unbounded and fixed-size cases hit this path; the JSON Schema
/// emitter encodes the length distinction.
fn is_byte_blob(field: &MsgField) -> bool {
    if !field.is_array {
        return false;
    }
    match &field.type_ref {
        MsgTypeRef::Primitive(p) => RosPrimitive::parse(p)
            .map(|prim| prim.is_blob_element())
            .unwrap_or(false),
        _ => false,
    }
}

fn emit_field_decl(out: &mut String, field: &MsgField, pkg: &str) {
    let type_str = match &field.type_ref {
        MsgTypeRef::Primitive(p) => {
            let blob = is_byte_blob(field);
            python_field_type(p, field.is_array && !blob, blob)
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
            let blob = is_byte_blob(field);
            python_default(p, field.is_array && !blob, blob)
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
    let blob = is_byte_blob(field);
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
    let blob = is_byte_blob(field);
    match &field.type_ref {
        MsgTypeRef::Primitive(_) if blob => {
            let _ = writeln!(
                out,
                "            {name}=_b64dec(d.get(\"{name}\", \"\")),",
                name = field.name
            );
        }
        MsgTypeRef::Primitive(p) => {
            let cast = python_cast(p);
            let default = python_default(p, field.is_array, false);
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

/// Build the JSON Schema fragment for one .msg field. The fragment is
/// emitted as Python source — for primitive scalars it's a literal
/// dict, for nested types it's a function call to the dependency's
/// `json_schema()`. The embedded constraints come from the parser:
///
///   - integer primitives → `minimum` / `maximum` from
///     `RosPrimitive::integer_range`
///   - fixed-size arrays → `minItems` = `maxItems` = N
///   - `string<=N` / `wstring<=N` → `maxLength` N
///   - `uint8[N]` blob → base64 string with `minLength`/`maxLength`
///     reflecting the encoded length
///   - trailing `# comment` → `description`
fn emit_json_schema_prop(out: &mut String, field: &MsgField, pkg: &str, depth: usize) {
    let indent = "    ".repeat(depth + 3);
    let blob = is_byte_blob(field);

    let mut entries: Vec<String> = Vec::new();
    match &field.type_ref {
        MsgTypeRef::Primitive(p) if blob => {
            entries.push(r#""type": "string""#.to_string());
            entries.push(r#""contentEncoding": "base64""#.to_string());
            // Fixed-size `uint8[N]` carries N bytes → base64-encoded
            // length is exactly ceil(N/3)*4 with up to two `=` pads.
            // We emit min == max so the LLM can't produce a shorter
            // or longer payload.
            if let Some(n) = field.array_size {
                let _ = p; // primitive identity already validated by is_byte_blob
                let encoded = ((n + 2) / 3) * 4;
                entries.push(format!(r#""minLength": {encoded}"#));
                entries.push(format!(r#""maxLength": {encoded}"#));
            }
        }
        MsgTypeRef::Primitive(p) => {
            let prim = parse_prim_or_panic(p);
            if field.is_array {
                let mut item_entries: Vec<String> =
                    vec![format!(r#""type": "{}""#, prim.json_schema_type())];
                push_integer_range(&mut item_entries, prim);
                push_string_max_len(&mut item_entries, prim, field.string_max_len);
                let item_json = format!("{{{}}}", item_entries.join(", "));
                entries.push(r#""type": "array""#.to_string());
                entries.push(format!(r#""items": {item_json}"#));
                push_array_bounds(&mut entries, field.array_size);
            } else {
                entries.push(format!(r#""type": "{}""#, prim.json_schema_type()));
                push_integer_range(&mut entries, prim);
                push_string_max_len(&mut entries, prim, field.string_max_len);
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
                // Note: we can't push items + size + description in
                // one literal dict because `items` is a runtime call.
                // We assemble the dict using a Python helper that
                // shallow-merges; keeps generated code uniform.
                let mut frag = format!(r#"{{"type": "array", "items": {qualified}"#);
                if let Some(n) = field.array_size {
                    frag.push_str(&format!(r#", "minItems": {n}, "maxItems": {n}"#));
                }
                if !field.description.is_empty() {
                    frag.push_str(&format!(
                        r#", "description": {}"#,
                        py_string_literal(&field.description)
                    ));
                }
                frag.push('}');
                let _ = writeln!(
                    out,
                    "{indent}\"{name}\": {frag},",
                    indent = indent,
                    name = field.name,
                );
                return;
            } else {
                // Same shape: nested object schema is a runtime call.
                let mut frag = qualified;
                if !field.description.is_empty() {
                    frag = format!(
                        "{{**{frag}, \"description\": {desc}}}",
                        frag = frag,
                        desc = py_string_literal(&field.description)
                    );
                }
                let _ = writeln!(
                    out,
                    "{indent}\"{name}\": {frag},",
                    indent = indent,
                    name = field.name,
                );
                return;
            }
        }
    }
    if !field.description.is_empty() {
        entries.push(format!(
            r#""description": {}"#,
            py_string_literal(&field.description)
        ));
    }
    let _ = writeln!(
        out,
        "{indent}\"{name}\": {{{body}}},",
        indent = indent,
        name = field.name,
        body = entries.join(", ")
    );
}

fn push_integer_range(entries: &mut Vec<String>, prim: RosPrimitive) {
    if let Some((lo, hi)) = prim.integer_range() {
        entries.push(format!(r#""minimum": {lo}"#));
        entries.push(format!(r#""maximum": {hi}"#));
    }
}

fn push_string_max_len(entries: &mut Vec<String>, prim: RosPrimitive, bound: Option<usize>) {
    if matches!(prim, RosPrimitive::String | RosPrimitive::Wstring)
        && let Some(n) = bound
    {
        entries.push(format!(r#""maxLength": {n}"#));
    }
}

fn push_array_bounds(entries: &mut Vec<String>, array_size: Option<usize>) {
    if let Some(n) = array_size {
        entries.push(format!(r#""minItems": {n}"#));
        entries.push(format!(r#""maxItems": {n}"#));
    }
}

/// Emit a Python string literal with the value of `s`, escaping
/// backslashes / double-quotes / control chars so the generated file
/// remains valid Python regardless of what's in the .msg comment.
fn py_string_literal(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '\\' => out.push_str(r"\\"),
            '"' => out.push_str("\\\""),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(&mut out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
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
        // Empty message → empty dataclass body (`pass`). Previously
        // we synthesized a `_empty: bool` field which leaked into
        // `__eq__` / `__repr__` and shadowed any user-declared field
        // named `_empty`. `pass` is the right "no fields" body for a
        // @dataclass.
        let _ = writeln!(out, "    pass");
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
        // additionalProperties=false even on empty messages so an
        // LLM can't sneak extra keys into a no-arg call.
        let _ = writeln!(
            out,
            "        return {{\"type\": \"object\", \"properties\": {{}}, \
             \"additionalProperties\": False}}"
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
    // Every field is required: ROS messages have no concept of
    // "optional" (every declared field has a default and must round-
    // trip), so an LLM should always provide every field. Listing
    // them as `required` makes that explicit to schema validators.
    let _ = write!(out, "            \"required\": [");
    let mut first = true;
    for f in &spec.fields {
        if !first {
            let _ = write!(out, ", ");
        }
        first = false;
        let _ = write!(out, "\"{}\"", f.name);
    }
    let _ = writeln!(out, "],");
    // Reject extra properties — an LLM passing a typo'd field name
    // is a bug, not silently-ignored noise.
    let _ = writeln!(out, "            \"additionalProperties\": False,");
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
        let raw_specs: Vec<_> = resolver
            .ordered_specs()
            .into_iter()
            .filter(|s| &s.package == package)
            .collect();

        if raw_specs.is_empty() {
            continue;
        }

        // Topo-sort same-package types so that a class whose default
        // factory references a sibling (`field(default_factory=lambda:
        // Vector3())`) is emitted AFTER its dependency. `from
        // __future__ import annotations` only defers type-annotation
        // resolution — the lambda body runs at call time and would
        // hit `NameError: Vector3 is not defined` if Vector3's class
        // statement hadn't executed yet.
        let specs = topo_sort_same_package(&raw_specs);

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
        // validate=True makes b64decode raise on garbage characters
        // instead of silently returning truncated/junk bytes. A
        // typo'd payload over MCP should fail loudly.
        let _ = writeln!(out, "    return base64.b64decode(s, validate=True) if s else b\"\"");
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

/// Order the specs of one package so dependencies come before
/// dependents. Cross-package references don't matter (they go through
/// `import other_mcp` and resolve at runtime via attribute lookup);
/// only same-package siblings need ordering.
///
/// On unresolvable cycle (A→B→A within one package — extremely rare in
/// real ROS IDL), falls back to the input order and lets Python raise
/// at instantiation time. We don't try to break cycles silently.
fn topo_sort_same_package<'a>(specs: &[&'a MsgSpec]) -> Vec<&'a MsgSpec> {
    if specs.is_empty() {
        return Vec::new();
    }
    let pkg = specs[0].package.clone();
    let by_name: BTreeMap<String, &MsgSpec> = specs
        .iter()
        .map(|s| (s.name.clone(), *s))
        .collect();

    enum Mark {
        Temp,
        Done,
    }
    let mut marks: BTreeMap<String, Mark> = BTreeMap::new();
    let mut order: Vec<&MsgSpec> = Vec::new();

    fn visit<'a>(
        name: &str,
        pkg: &str,
        by_name: &BTreeMap<String, &'a MsgSpec>,
        marks: &mut BTreeMap<String, Mark>,
        order: &mut Vec<&'a MsgSpec>,
    ) {
        match marks.get(name) {
            Some(Mark::Done) => return,
            Some(Mark::Temp) => return, // cycle — give up on ordering
            None => {}
        }
        let Some(spec) = by_name.get(name) else { return; };
        marks.insert(name.to_string(), Mark::Temp);
        for f in &spec.fields {
            if let MsgTypeRef::Named { package, name: dep_name } = &f.type_ref
                && package == pkg
            {
                visit(dep_name, pkg, by_name, marks, order);
            }
        }
        marks.insert(name.to_string(), Mark::Done);
        order.push(*spec);
    }

    for spec in specs {
        visit(&spec.name, &pkg, &by_name, &mut marks, &mut order);
    }
    order
}
