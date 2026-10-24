// SPDX-License-Identifier: MulanPSL-2.0
use serde_json::Value;
use std::collections::HashMap;

const MAX_DEPTH: usize = 64;

pub fn is_reference(obj: &serde_json::Map<String, Value>) -> bool {
    obj.contains_key("var") && obj.keys().all(|key| key == "var" || key == "path")
}

fn is_operation(obj: &serde_json::Map<String, Value>) -> bool {
    obj.len() == 2 && obj.contains_key("op") && obj.contains_key("args")
}

pub fn validate_expr(expr: &Value) -> Result<(), String> {
    validate(expr, 0)
}

fn validate(expr: &Value, depth: usize) -> Result<(), String> {
    if depth > MAX_DEPTH {
        return Err("expression nesting exceeds 64 levels".into());
    }
    match expr {
        Value::Object(obj) if is_reference(obj) => {
            if obj["var"]
                .as_str()
                .is_none_or(|name| name.trim().is_empty())
            {
                return Err("var must be a non-empty string".into());
            }
            if let Some(path) = obj.get("path") {
                let parts = path.as_array().ok_or("path must be an array")?;
                if parts
                    .iter()
                    .any(|part| !part.is_string() && part.as_u64().is_none())
                {
                    return Err("path entries must be field names or array indices".into());
                }
            }
        }
        Value::Object(obj) if is_operation(obj) => {
            let op = obj["op"].as_str().ok_or("expression op must be a string")?;
            let args = obj
                .get("args")
                .and_then(Value::as_array)
                .ok_or("expression args must be an array")?;
            let valid = match op {
                "eq" | "ne" | "lt" | "le" | "gt" | "ge" => args.len() == 2,
                "not" => args.len() == 1,
                "and" | "or" => !args.is_empty(),
                _ => return Err(format!("unknown expression operator '{op}'")),
            };
            if !valid {
                return Err(format!("invalid argument count for '{op}'"));
            }
            for arg in args {
                validate(arg, depth + 1)?;
            }
        }
        Value::Object(obj) => {
            for value in obj.values() {
                validate(value, depth + 1)?;
            }
        }
        Value::Array(items) => {
            for value in items {
                validate(value, depth + 1)?;
            }
        }
        _ => {}
    }
    Ok(())
}

pub fn eval_value(expr: &Value, vars: &HashMap<String, Value>) -> Result<Value, String> {
    validate_expr(expr)?;
    eval(expr, vars)
}

fn eval(expr: &Value, vars: &HashMap<String, Value>) -> Result<Value, String> {
    match expr {
        Value::Object(obj) if is_reference(obj) => {
            let name = obj["var"].as_str().unwrap();
            let mut value = vars
                .get(name)
                .ok_or_else(|| format!("undefined variable '{name}'"))?;
            if let Some(parts) = obj.get("path").and_then(Value::as_array) {
                for part in parts {
                    value = if let Some(field) = part.as_str() {
                        value.as_object().and_then(|fields| fields.get(field))
                    } else {
                        part.as_u64()
                            .and_then(|index| usize::try_from(index).ok())
                            .and_then(|index| value.as_array().and_then(|items| items.get(index)))
                    }
                    .ok_or_else(|| format!("missing path entry {part} in variable '{name}'"))?;
                }
            }
            Ok(value.clone())
        }
        Value::Object(obj) if is_operation(obj) => {
            let op = obj["op"].as_str().unwrap();
            let args = obj["args"].as_array().unwrap();
            if op == "and" || op == "or" {
                for arg in args {
                    let value = eval_bool(arg, vars)?;
                    if value == (op == "or") {
                        return Ok(Value::Bool(value));
                    }
                }
                return Ok(Value::Bool(op == "and"));
            }
            if op == "not" {
                return Ok(Value::Bool(!eval_bool(&args[0], vars)?));
            }
            let left = eval(&args[0], vars)?;
            let right = eval(&args[1], vars)?;
            let value = match op {
                "eq" => left == right,
                "ne" => left != right,
                _ => {
                    let order = if let (Some(a), Some(b)) = (left.as_i64(), right.as_i64()) {
                        a.partial_cmp(&b)
                    } else if let (Some(a), Some(b)) = (left.as_u64(), right.as_u64()) {
                        a.partial_cmp(&b)
                    } else if let (Some(a), Some(b)) = (left.as_f64(), right.as_f64()) {
                        a.partial_cmp(&b)
                    } else if let (Some(a), Some(b)) = (left.as_str(), right.as_str()) {
                        a.partial_cmp(b)
                    } else {
                        None
                    }
                    .ok_or("ordered comparison requires two numbers or two strings")?;
                    match op {
                        "lt" => order.is_lt(),
                        "le" => !order.is_gt(),
                        "gt" => order.is_gt(),
                        "ge" => !order.is_lt(),
                        _ => unreachable!("validated operator"),
                    }
                }
            };
            Ok(Value::Bool(value))
        }
        Value::Object(obj) => obj
            .iter()
            .map(|(key, value)| Ok((key.clone(), eval(value, vars)?)))
            .collect::<Result<serde_json::Map<_, _>, String>>()
            .map(Value::Object),
        Value::Array(items) => items
            .iter()
            .map(|value| eval(value, vars))
            .collect::<Result<Vec<_>, _>>()
            .map(Value::Array),
        _ => Ok(expr.clone()),
    }
}

fn eval_bool(expr: &Value, vars: &HashMap<String, Value>) -> Result<bool, String> {
    eval(expr, vars)?
        .as_bool()
        .ok_or_else(|| "condition must evaluate to a boolean".into())
}

pub fn eval_condi(expr: &Value, vars: &HashMap<String, Value>) -> Result<bool, String> {
    validate_expr(expr)?;
    eval_bool(expr, vars)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn resolves_nested_arguments_and_rejects_missing_values() {
        let vars = HashMap::from([("camera".into(), json!({"items":[{"found":true}]}))]);
        assert_eq!(
            eval_value(
                &json!({"found":{"var":"camera","path":["items",0,"found"]}}),
                &vars
            )
            .unwrap(),
            json!({"found":true})
        );
        assert!(eval_value(&json!({"var":"missing"}), &vars).is_err());
        assert!(eval_value(&json!({"var":"camera","path":["unknown"]}), &vars).is_err());
    }

    #[test]
    fn ordinary_capability_objects_keep_their_op_and_var_fields() {
        let value = json!({"op":"move","direction":"left","options":{"var":"setting","value":3}});
        assert_eq!(eval_value(&value, &HashMap::new()).unwrap(), value);
    }

    #[test]
    fn conditions_are_strict_and_short_circuit() {
        let vars = HashMap::new();
        assert!(eval_condi(&json!({"op":"or","args":[true,{"var":"missing"}]}), &vars).unwrap());
        assert!(!eval_condi(&json!({"op":"and","args":[false,{"var":"missing"}]}), &vars).unwrap());
        assert!(eval_condi(&json!(1), &vars).is_err());
        assert!(eval_condi(&json!({"op":"gt","args":[3,2]}), &vars).unwrap());
        assert!(validate_expr(&json!({"op":"eq","args":[1]})).is_err());
    }
}
