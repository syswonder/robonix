// SPDX-License-Identifier: MulanPSL-2.0
//! A package's `config.spec`, in its strict form.
//!
//! `config.spec` documents what a package's `config:` accepts. It may be
//! free text. When it is YAML with `specVersion: 1` at the top it is strict:
//! a JSON Schema subset that tools (the hub's deployment editor among them)
//! build forms from, so it has to be exactly that subset. Robonix's own
//! system and bundled services ship strict specs.
//!
//! The subset, at the top: `specVersion`, `description`, `properties`,
//! `required`. For each property: `type` (string, integer, number, boolean,
//! array, object, null, or a list of them; required), `description`,
//! `default`, `enum`, `minimum` and `maximum` (both inclusive), `items`,
//! `properties`, `required`, and three extensions: `x-secret` (a credential,
//! to be written as `${VAR}`), `x-group` (a heading the field is shown under)
//! and `x-provider` (the value names another entry of the deployment: the
//! contract it must provide, or `true` for any). Anything else a field needs
//! to say (units, environment fallbacks, defaults computed at run time,
//! conditions, deprecation) goes in its description. `status` is the
//! deployment's switch, read by rbnx, and is never a property.

use anyhow::{Result, bail};
use serde_yaml::Value;

const TOP: &[&str] = &["specVersion", "description", "properties", "required"];
const FIELD: &[&str] = &[
    "type",
    "description",
    "default",
    "enum",
    "minimum",
    "maximum",
    "items",
    "properties",
    "required",
    "x-secret",
    "x-group",
    "x-provider",
];
const TYPES: &[&str] = &[
    "string", "integer", "number", "boolean", "array", "object", "null",
];

/// `Ok(false)` for a free-text spec, `Ok(true)` for a strict one that keeps
/// to the subset, and the first departure from it otherwise.
pub fn check(text: &str) -> Result<bool> {
    let Ok(root) = serde_yaml::from_str::<Value>(text) else {
        return Ok(false);
    };
    let Some(version) = root.get("specVersion") else {
        return Ok(false);
    };
    if version.as_u64() != Some(1) {
        bail!("specVersion must be 1, not {version:?}");
    }
    keys(&root, TOP, "top level")?;
    object(&root, "config")?;
    Ok(true)
}

fn keys(node: &Value, allowed: &[&str], at: &str) -> Result<()> {
    let Some(map) = node.as_mapping() else {
        bail!("{at}: must be a mapping");
    };
    for k in map.keys() {
        let k = k.as_str().unwrap_or_default();
        if !allowed.contains(&k) {
            bail!("{at}: `{k}` is not part of the strict config.spec format");
        }
    }
    Ok(())
}

/// The `properties` and `required` of an object, at the top or nested.
fn object(node: &Value, at: &str) -> Result<()> {
    let props = match node.get("properties") {
        None => serde_yaml::Mapping::new(),
        Some(Value::Mapping(m)) => m.clone(),
        Some(_) => bail!("{at}.properties: must be a mapping"),
    };
    for (k, field) in &props {
        let name = k.as_str().unwrap_or_default();
        if name == "status" && at == "config" {
            bail!(
                "config.status: `status` is the deployment's switch, read by rbnx; it is not a config property"
            );
        }
        property(field, &format!("{at}.{name}"))?;
    }
    if let Some(required) = node.get("required") {
        let Some(list) = required.as_sequence() else {
            bail!("{at}.required: must be a list");
        };
        for r in list {
            let r = r.as_str().unwrap_or_default();
            if !props.contains_key(r) {
                bail!("{at}.required: `{r}` is not one of its properties");
            }
        }
    }
    Ok(())
}

fn property(field: &Value, at: &str) -> Result<()> {
    keys(field, FIELD, at)?;
    let types: Vec<&str> = match field.get("type") {
        None => bail!("{at}.type: every property declares its type"),
        Some(Value::String(t)) => vec![t.as_str()],
        Some(Value::Sequence(ts)) => ts.iter().map(|t| t.as_str().unwrap_or("?")).collect(),
        Some(t) => bail!("{at}.type: must be a name or a list of names, not {t:?}"),
    };
    for t in &types {
        if !TYPES.contains(t) {
            bail!("{at}.type: unknown type `{t}`");
        }
    }
    if let Some(desc) = field.get("description")
        && !desc.is_string()
    {
        bail!("{at}.description: must be text");
    }
    for bound in ["minimum", "maximum"] {
        if let Some(b) = field.get(bound)
            && b.as_f64().is_none()
        {
            bail!("{at}.{bound}: must be a number");
        }
    }
    if let Some(secret) = field.get("x-secret")
        && !secret.is_bool()
    {
        bail!("{at}.x-secret: must be true or false");
    }
    match field.get("x-provider") {
        None => {}
        Some(Value::Bool(true)) => {}
        Some(Value::String(c)) if !c.trim().is_empty() => {}
        Some(p) => bail!("{at}.x-provider: a contract ID or true, not {p:?}"),
    }
    if field.get("x-provider").is_some() && !types.contains(&"string") {
        bail!("{at}.x-provider: only a string names a provider");
    }
    if let Some(items) = field.get("items") {
        property(items, &format!("{at}.items"))?;
    }
    if field.get("properties").is_some() || field.get("required").is_some() {
        object(field, at)?;
    }
    let choices = match field.get("enum") {
        None => None,
        Some(Value::Sequence(c)) if !c.is_empty() => Some(c),
        Some(_) => bail!("{at}.enum: must be a non-empty list"),
    };
    if let Some(choices) = choices {
        for c in choices {
            if !fits(c, &types) {
                bail!("{at}.enum: {c:?} is not of type {types:?}");
            }
        }
    }
    if let Some(default) = field.get("default") {
        if !fits(default, &types) {
            bail!("{at}.default: {default:?} is not of type {types:?}");
        }
        if let Some(choices) = choices
            && !default.is_null()
            && !choices.contains(default)
        {
            bail!("{at}.default: {default:?} is not one of its enum");
        }
        if let Some(n) = default.as_f64() {
            let bound = |k: &str| field.get(k).and_then(Value::as_f64);
            let out =
                bound("minimum").is_some_and(|m| n < m) || bound("maximum").is_some_and(|m| n > m);
            if out {
                bail!("{at}.default: {n} is outside its own bounds");
            }
        }
    }
    Ok(())
}

/// Whether a value is of one of the declared types.
fn fits(v: &Value, types: &[&str]) -> bool {
    types.iter().any(|t| match *t {
        "string" => v.is_string(),
        "integer" => v.as_i64().is_some() || v.as_u64().is_some(),
        "number" => v.as_f64().is_some(),
        "boolean" => v.is_bool(),
        "array" => v.is_sequence(),
        "object" => v.is_mapping(),
        "null" => v.is_null(),
        _ => false,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::{Path, PathBuf};

    #[test]
    fn free_text_is_left_alone_and_the_subset_is_held_to() {
        assert!(!check("Set `rate` to the sensor's rate.").unwrap());
        assert!(!check("config:\n  rate: 10\n").unwrap());
        assert!(
            check(
                "specVersion: 1\nproperties:\n  rate: {type: integer, default: 10, minimum: 1}\n  cam: {type: string, x-provider: robonix/primitive/camera/rgb}\n"
            )
            .unwrap()
        );
        for bad in [
            "specVersion: 2\n",
            "specVersion: 1\nschema: {}\n",
            "specVersion: 1\nproperties:\n  rate: {type: int}\n",
            "specVersion: 1\nproperties:\n  rate: {type: integer, default: fast}\n",
            "specVersion: 1\nproperties:\n  rate: {type: integer, default: 0, minimum: 1}\n",
            "specVersion: 1\nproperties:\n  mode: {type: string, enum: [a, b], default: c}\n",
            "specVersion: 1\nproperties:\n  rate: {type: integer, pattern: x}\n",
            "specVersion: 1\nproperties:\n  rate: {type: integer, exclusiveMinimum: 0}\n",
            "specVersion: 1\nproperties:\n  rate: {default: 1}\n",
            "specVersion: 1\nproperties:\n  cam: {type: integer, x-provider: robonix/primitive/camera/rgb}\n",
            "specVersion: 1\nproperties:\n  cam: {type: string, x-provider: false}\n",
            "specVersion: 1\nproperties:\n  status: {type: string}\n",
            "specVersion: 1\nproperties: {}\nrequired: [rate]\n",
        ] {
            assert!(check(bad).is_err(), "{bad}");
        }
    }

    /// Robonix's own services ship strict specs, and they keep to the format.
    #[test]
    fn robonix_services_ship_strict_specs() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let mut specs: Vec<PathBuf> = Vec::new();
        for entry in std::fs::read_dir(root.join("system")).unwrap().flatten() {
            let spec = entry.path().join("config.spec");
            if spec.is_file() {
                specs.push(spec);
            }
        }
        let mut stack = vec![root.join("services")];
        while let Some(dir) = stack.pop() {
            if dir.join("package_manifest.yaml").is_file() {
                specs.push(dir.join("config.spec"));
            } else {
                stack.extend(
                    std::fs::read_dir(&dir)
                        .unwrap()
                        .flatten()
                        .map(|e| e.path())
                        .filter(|p| p.is_dir()),
                );
            }
        }
        assert!(specs.len() >= 13, "{specs:?}");
        for spec in specs {
            let text = std::fs::read_to_string(&spec)
                .unwrap_or_else(|_| panic!("{} is missing", spec.display()));
            match check(&text) {
                Ok(true) => {}
                Ok(false) => panic!("{} is not a strict config.spec", spec.display()),
                Err(e) => panic!("{}: {e}", spec.display()),
            }
        }
    }
}
