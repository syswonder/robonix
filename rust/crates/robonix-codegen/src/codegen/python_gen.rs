// SPDX-License-Identifier: MulanPSL-2.0
// Python ctypes code generator — ROS .msg -> Python ctypes.Structure
//
// Generates one Python file per ROS package, containing:
//   - ctypes.Structure subclasses for each .msg type
//   - Class-level integer constants from ROS IDL constant lines
//   - type_name() staticmethod required by the iceoryx2 Python API

use anyhow::{Context, Result};
use std::collections::BTreeSet;
use std::fmt::Write as FmtWrite;
use std::fs;
use std::path::Path;

use super::msg_parser::{MsgField, MsgResolver, MsgSpec, MsgTypeRef};

// ── Type mapping ─────────────────────────────────────────────────────────────

fn ros_primitive_to_ctypes(ros_type: &str) -> &'static str {
    match ros_type {
        "bool" => "ctypes.c_bool",
        "byte" | "uint8" => "ctypes.c_uint8",
        "char" | "int8" => "ctypes.c_int8",
        "uint16" => "ctypes.c_uint16",
        "int16" => "ctypes.c_int16",
        "uint32" => "ctypes.c_uint32",
        "int32" => "ctypes.c_int32",
        "uint64" => "ctypes.c_uint64",
        "int64" => "ctypes.c_int64",
        "float32" => "ctypes.c_float",
        "float64" => "ctypes.c_double",
        "string" | "wstring" => "ctypes.c_char * 256",
        _ => "ctypes.c_uint8",
    }
}

// ── Constant extraction ───────────────────────────────────────────────────────

/// Extract constant definitions from a .msg file.
/// Returns `(ros_type, NAME, value_str)` for lines like `uint8 RGB8 = 0`.
fn extract_constants(path: &Path) -> Vec<(String, String, String)> {
    let Ok(content) = fs::read_to_string(path) else {
        return vec![];
    };
    let mut constants = Vec::new();
    for raw_line in content.lines() {
        let line = raw_line.split('#').next().unwrap_or_default().trim();
        if line.is_empty() || !line.contains('=') {
            continue;
        }
        let mut parts = line.splitn(2, '=');
        let lhs = parts.next().unwrap_or("").trim();
        let rhs = parts.next().unwrap_or("").trim().to_string();
        if rhs.is_empty() {
            continue;
        }
        let mut lhs_parts = lhs.split_whitespace();
        let Some(type_str) = lhs_parts.next() else {
            continue;
        };
        let Some(name) = lhs_parts.next() else {
            continue;
        };
        if name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
            constants.push((type_str.to_string(), name.to_string(), rhs));
        }
    }
    constants
}

// ── Field flattening ──────────────────────────────────────────────────────────

/// A resolved (field_name, ctypes_type_str) pair ready for `_fields_`.
/// `comment` (if set) is emitted on its own line before the tuple entry.
struct FlatField {
    name: String,
    ctypes_type: String,
    comment: Option<String>,
}

/// Recursively flatten a single `MsgField` into zero or more `FlatField`s.
///
/// Cross-package and same-package nested message types are both flattened:
/// a field `header` of type `std_msgs/Header` expands to
/// `header_stamp_sec`, `header_stamp_nanosec`, `header_frame_id`, etc.
fn flatten_field(
    field: &MsgField,
    prefix: &str,
    resolver: &MsgResolver,
    out: &mut Vec<FlatField>,
    depth: usize,
) {
    if depth > 10 {
        out.push(FlatField {
            name: prefix.to_string(),
            ctypes_type: "ctypes.c_uint8".to_string(),
            comment: Some(format!("recursion limit reached at '{}'", prefix)),
        });
        return;
    }

    match &field.type_ref {
        MsgTypeRef::Primitive(p) => {
            if field.is_array {
                match field.array_size {
                    Some(n) => {
                        // Fixed-size array: use ctypes array type
                        let base = match p.as_str() {
                            "string" | "wstring" => "ctypes.c_char * 256",
                            other => ros_primitive_to_ctypes(other),
                        };
                        let ct = if p == "string" || p == "wstring" {
                            format!("ctypes.c_uint8 * {}", n * 256)
                        } else {
                            format!("{} * {}", base, n)
                        };
                        out.push(FlatField {
                            name: prefix.to_string(),
                            ctypes_type: ct,
                            comment: None,
                        });
                    }
                    None => {
                        // Unbounded array — cap at a fixed max so the struct has a
                        // fixed size compatible with iceoryx2. Callers must ensure
                        // actual payload fits within the cap.
                        let (cap_expr, cap_note) = if p == "uint8" || p == "byte" {
                            (
                                "_MAX_IMAGE_BYTES",
                                format!("unbounded {}[]; capped at _MAX_IMAGE_BYTES", p),
                            )
                        } else {
                            (
                                "_MAX_ARRAY_ELEMS",
                                format!("unbounded {}[]; capped at _MAX_ARRAY_ELEMS", p),
                            )
                        };
                        let base = ros_primitive_to_ctypes(p);
                        out.push(FlatField {
                            name: prefix.to_string(),
                            ctypes_type: format!("{} * {}", base, cap_expr),
                            comment: Some(cap_note),
                        });
                    }
                }
            } else if p == "string" || p == "wstring" {
                out.push(FlatField {
                    name: prefix.to_string(),
                    ctypes_type: "ctypes.c_char * 256".to_string(),
                    comment: None,
                });
            } else {
                out.push(FlatField {
                    name: prefix.to_string(),
                    ctypes_type: ros_primitive_to_ctypes(p).to_string(),
                    comment: None,
                });
            }
        }

        MsgTypeRef::Named { package, name } => {
            if field.is_array {
                // Named message arrays can't be cleanly flattened into a C struct.
                // Emit as a raw byte blob so the field is at least present and sized.
                let cap = match field.array_size {
                    Some(n) => format!("{}", n * 64), // rough per-element estimate
                    None => "_MAX_ARRAY_ELEMS * 64".to_string(),
                };
                out.push(FlatField {
                    name: prefix.to_string(),
                    ctypes_type: format!("ctypes.c_uint8 * {}", cap),
                    comment: Some(format!(
                        "opaque blob: {}/{}[] not flattenable",
                        package, name
                    )),
                });
                return;
            }

            // Try to resolve and flatten the nested type
            let key = (package.clone(), name.clone());
            if let Some(nested) = resolver.cache.get(&key) {
                if nested.fields.is_empty() {
                    // Empty message (possibly only constants) — emit a placeholder byte
                    out.push(FlatField {
                        name: format!("{}_", prefix),
                        ctypes_type: "ctypes.c_uint8".to_string(),
                        comment: Some(format!("placeholder: {}/{} is empty", package, name)),
                    });
                } else {
                    for sub in &nested.fields {
                        let sub_prefix = format!("{}_{}", prefix, sub.name);
                        flatten_field(sub, &sub_prefix, resolver, out, depth + 1);
                    }
                }
            } else {
                // Unresolvable — emit opaque byte
                out.push(FlatField {
                    name: prefix.to_string(),
                    ctypes_type: "ctypes.c_uint8".to_string(),
                    comment: Some(format!("unresolved type: {}/{}", package, name)),
                });
            }
        }
    }
}

// ── Class emitter ─────────────────────────────────────────────────────────────

fn emit_class(out: &mut String, spec: &MsgSpec, resolver: &MsgResolver) {
    let _ = writeln!(out, "class {}(ctypes.Structure):", spec.name);

    // ── Constants (from =  lines in .msg) ───────────────────────────────
    let constants = resolver
        .index
        .get(&(spec.package.clone(), spec.name.clone()))
        .map(|p| extract_constants(p))
        .unwrap_or_default();

    for (_, cname, cval) in &constants {
        let _ = writeln!(out, "    {} = {}", cname, cval.trim());
    }
    if !constants.is_empty() {
        let _ = writeln!(out);
    }

    // ── _fields_ ────────────────────────────────────────────────────────
    let mut flat: Vec<FlatField> = Vec::new();
    for field in &spec.fields {
        flatten_field(field, &field.name, resolver, &mut flat, 0);
    }

    if flat.is_empty() {
        let _ = writeln!(out, "    _fields_: list = []");
    } else {
        let _ = writeln!(out, "    _fields_ = [");
        for f in &flat {
            if let Some(ref cmt) = f.comment {
                let _ = writeln!(out, "        # {}", cmt);
            }
            let _ = writeln!(out, "        (\"{}\", {}),", f.name, f.ctypes_type);
        }
        let _ = writeln!(out, "    ]");
    }

    // ── type_name() — required by iceoryx2 Python API ───────────────────
    let _ = writeln!(out);
    let _ = writeln!(out, "    @staticmethod");
    let _ = writeln!(out, "    def type_name() -> str:");
    let _ = writeln!(
        out,
        "        return \"robonix.{}.{}\"",
        spec.package, spec.name
    );
}

// Default max element counts for unbounded arrays.
// These become module-level constants in the generated Python file so callers
// can override them before first use if their payloads are larger.
const UNBOUNDED_UINT8_MAX: usize = 1920 * 1080 * 4; // 1080p RGBA ≈ 8 MiB
const UNBOUNDED_ELEM_MAX: usize = 256; // all other primitive element types

// ── Public entry point ────────────────────────────────────────────────────────

pub fn generate(resolver: &MsgResolver, out_dir: &Path) -> Result<()> {
    fs::create_dir_all(out_dir)?;

    let mut all_packages = BTreeSet::new();
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

        let mut out = String::new();
        let _ = writeln!(
            out,
            "# @generated by robonix-codegen --lang python — DO NOT EDIT"
        );
        let _ = writeln!(out, "# source: ROS IDL package '{}'", package);
        let _ = writeln!(
            out,
            "# Payload types for the iceoryx2 Python API (ctypes.Structure + type_name())"
        );
        let _ = writeln!(
            out,
            "# Re-generate: cargo run -p robonix-codegen -- --lang python -I <lib> -o <out>"
        );
        let _ = writeln!(out);
        let _ = writeln!(out, "import ctypes");
        let _ = writeln!(out);
        // Module-level max sizes for unbounded arrays — override before first use.
        let _ = writeln!(
            out,
            "# Max element counts for unbounded (variable-length) array fields."
        );
        let _ = writeln!(
            out,
            "# iceoryx2 requires fixed-size payloads; these caps are the upper bound."
        );
        let _ = writeln!(
            out,
            "# Override the constant in your code before importing these types if needed."
        );
        let _ = writeln!(
            out,
            "_MAX_IMAGE_BYTES = {}  # 1080p RGBA upper bound",
            UNBOUNDED_UINT8_MAX
        );
        let _ = writeln!(
            out,
            "_MAX_ARRAY_ELEMS = {}  # generic upper bound for non-byte arrays",
            UNBOUNDED_ELEM_MAX
        );
        let _ = writeln!(out);
        let _ = writeln!(out);

        for spec in &specs {
            emit_class(&mut out, spec, resolver);
            let _ = writeln!(out);
            let _ = writeln!(out);
        }

        let filename = format!("{}_iox2.py", package);
        let filepath = out_dir.join(&filename);
        fs::write(&filepath, &out)
            .with_context(|| format!("failed to write '{}'", filepath.display()))?;
        eprintln!(
            "[robonix-codegen] python: '{}' ({} msgs) -> {}",
            package,
            specs.len(),
            filepath.display()
        );
    }

    // Write package __init__.py so the directory is importable as a Python package
    let init = out_dir.join("__init__.py");
    if !init.exists() {
        fs::write(&init, "# @generated by robonix-codegen --lang python\n")?;
    }

    Ok(())
}
