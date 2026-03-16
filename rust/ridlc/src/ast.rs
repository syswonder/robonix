// SPDX-License-Identifier: MulanPSL-2.0
// RIDL AST types (RFC001 subset)

use anyhow::{Context, Result, bail};

/// Full RIDL file AST; multiple files merge by namespace.
#[derive(Default)]
pub struct File {
    pub namespace: Option<String>,
    pub imports: Vec<Import>,
    pub interfaces: Vec<Interface>,
}

/// Resolve type_ref using imports: full name (contains '/') is unchanged;
/// short name is expanded via import. If multiple imports match (conflict), error.
fn resolve_type_ref(type_ref: &str, imports: &[Import]) -> Result<String> {
    let trimmed = type_ref.trim();
    if trimmed.is_empty() {
        bail!("empty type reference");
    }
    // Full name: contains '/'
    if trimmed.contains('/') {
        return Ok(trimmed.to_string());
    }
    // Short name: strip [] suffix, resolve base, re-append
    let (base, suffix) = if trimmed.ends_with("[]") {
        (trimmed[..trimmed.len() - 2].trim_end(), "[]")
    } else {
        (trimmed, "")
    };
    let matches: Vec<&Import> = imports
        .iter()
        .filter(|imp| !imp.wildcard)
        .filter(|imp| {
            imp.path.rsplit('/').next().map(|s| s == base).unwrap_or(false)
        })
        .collect();
    match matches.len() {
        0 => bail!(
            "type '{}' not found in imports (use full name like pkg/msg/Name)",
            base
        ),
        1 => Ok(format!("{}{}", matches[0].path, suffix)),
        _ => {
            // Deduplicate by path: duplicate imports (same path) are not ambiguous
            let paths: std::collections::HashSet<_> =
                matches.iter().map(|m| m.path.as_str()).collect();
            if paths.len() == 1 {
                Ok(format!("{}{}", matches[0].path, suffix))
            } else {
                bail!(
                    "type '{}' ambiguous: multiple imports match (use full name)",
                    base
                )
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct Import {
    pub path: String,   // e.g. "geometry_msgs/msg/Twist"
    pub wildcard: bool, // import pkg/msg/*
}

#[derive(Clone, Debug)]
pub enum Interface {
    Stream(StreamDef),
    Command(CommandDef),
    Query(QueryDef),
}

#[derive(Clone, Debug)]
pub struct StreamDef {
    pub name: String,
    pub annotations: Vec<Annotation>,
    pub fields: Vec<StreamField>,
    pub version: Option<String>,
}

#[derive(Clone, Debug)]
pub enum StreamDirection {
    Input,
    Output,
}

#[derive(Clone, Debug)]
pub struct StreamField {
    pub direction: StreamDirection,
    pub name: String,
    pub type_ref: String,
    pub annotations: Vec<Annotation>,
}

#[derive(Clone, Debug)]
pub struct Annotation {
    pub key: String,
    pub value: Option<String>,
}

#[derive(Clone, Debug)]
pub struct CommandDef {
    pub name: String,
    pub annotations: Vec<Annotation>,
    pub input: Option<CommandField>,
    pub output: Option<CommandField>,
    pub result: Option<CommandField>,
    pub version: Option<String>,
    pub safety: Vec<SafetyItem>,
}

#[derive(Clone, Debug)]
pub struct CommandField {
    pub name: String,
    pub type_ref: String,
    pub annotations: Vec<Annotation>,
}

#[derive(Clone, Debug)]
pub struct SafetyItem {
    pub name: String,
    pub safety_type: String,
    pub rw: bool, // true = RW, false = RO
}

#[derive(Clone, Debug)]
pub struct QueryDef {
    pub name: String,
    pub annotations: Vec<Annotation>,
    pub request: QueryField,
    pub response: QueryField,
    pub version: Option<String>,
}

#[derive(Clone, Debug)]
pub struct QueryField {
    pub name: String,
    pub type_ref: String,
    pub annotations: Vec<Annotation>,
}

impl File {
    pub fn merge(&mut self, other: File) {
        if self.namespace.is_none() && other.namespace.is_some() {
            self.namespace = other.namespace;
        }
        self.imports.extend(other.imports);
        self.interfaces.extend(other.interfaces);
    }

    pub fn namespace_path(&self) -> Option<&str> {
        self.namespace.as_deref()
    }

    /// Resolve short type names to full paths using imports.
    /// After import pkg/msg/Name, field type_ref "Name" or "Name[]" becomes "pkg/msg/Name" or "pkg/msg/Name[]".
    pub fn resolve_imports(&mut self) -> Result<()> {
        for iface in &mut self.interfaces {
            match iface {
                Interface::Query(q) => {
                    q.request.type_ref =
                        resolve_type_ref(&q.request.type_ref, &self.imports)
                            .with_context(|| format!("query {} request", q.name))?;
                    q.response.type_ref =
                        resolve_type_ref(&q.response.type_ref, &self.imports)
                            .with_context(|| format!("query {} response", q.name))?;
                }
                Interface::Stream(s) => {
                    for f in &mut s.fields {
                        f.type_ref = resolve_type_ref(&f.type_ref, &self.imports)
                            .with_context(|| format!("stream {} field {}", s.name, f.name))?;
                    }
                }
                Interface::Command(c) => {
                    if let Some(ref mut inp) = c.input {
                        inp.type_ref = resolve_type_ref(&inp.type_ref, &self.imports)
                            .with_context(|| format!("command {} input", c.name))?;
                    }
                    if let Some(ref mut out) = c.output {
                        out.type_ref = resolve_type_ref(&out.type_ref, &self.imports)
                            .with_context(|| format!("command {} output", c.name))?;
                    }
                    if let Some(ref mut res) = c.result {
                        res.type_ref = resolve_type_ref(&res.type_ref, &self.imports)
                            .with_context(|| format!("command {} result", c.name))?;
                    }
                }
            }
        }
        Ok(())
    }
}
