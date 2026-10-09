use serde_json::Value;
use std::collections::HashMap;

pub fn validate_expr(expr: &Value) -> Result<(),String>;

pub fn eval_value(
    expr: &Value,
    vars: &HashMap<String, Value>,
) -> Result<Value, String>;

pub fn eval_condi(
    expr: &Value,
    vars: &HashMap<String, Value>,
) -> Result<bool, String>;