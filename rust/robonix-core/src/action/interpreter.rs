// SPDX-License-Identifier: MulanPSL-2.0
// DSL Interpreter Module
//
// This module parses DSL code into executable instructions.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// DSL instruction - simple skill call
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DSLInstruction {
    pub skill_name: String,
    pub inputs: HashMap<String, String>, // Parameter name -> value (as string, will be parsed)
}

// Parsed DSL program
#[derive(Debug, Clone)]
pub struct DSLProgram {
    pub instructions: Vec<DSLInstruction>,
}

/// DSL Interpreter
pub struct DSLInterpreter {
    // Future: could add configuration for DSL syntax
}

impl DSLInterpreter {
    pub fn new() -> Self {
        Self {}
    }

    /// Parse DSL code into a program
    pub fn parse_dsl(&self, dsl_code: &str) -> Result<DSLProgram, String> {
        // Simple temporary DSL parser
        // Format: skill_name(param1=value1, param2=value2)
        // Or: sequence(instruction1, instruction2)
        // Or: parallel(instruction1, instruction2)

        let mut instructions = Vec::new();

        // Split by lines and parse each
        for line in dsl_code.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue; // Skip empty lines and comments
            }

            // Try to parse as skill call: skill_name(param1=value1, param2=value2)
            if let Some(instruction) = self.parse_skill_call(line)? {
                instructions.push(instruction);
            }
        }

        Ok(DSLProgram { instructions })
    }

    fn parse_skill_call(&self, line: &str) -> Result<Option<DSLInstruction>, String> {
        // Simple parser for: skill_name(param1=value1, param2=value2)
        if let Some(open_paren) = line.find('(') {
            let skill_name = line[..open_paren].trim().to_string();
            if let Some(close_paren) = line.rfind(')') {
                let params_str = &line[open_paren + 1..close_paren];
                let mut inputs = HashMap::new();

                // Parse parameters
                for param in params_str.split(',') {
                    let param = param.trim();
                    if param.is_empty() {
                        continue;
                    }
                    if let Some(eq_pos) = param.find('=') {
                        let key = param[..eq_pos].trim().to_string();
                        let value = param[eq_pos + 1..].trim().to_string();
                        // Remove quotes if present
                        let value = value.trim_matches('"').trim_matches('\'').to_string();
                        inputs.insert(key, value);
                    }
                }

                return Ok(Some(DSLInstruction {
                    skill_name,
                    inputs,
                }));
            }
        }

        // If no parentheses, treat as simple skill name
        if !line.is_empty() {
            return Ok(Some(DSLInstruction {
                skill_name: line.to_string(),
                inputs: HashMap::new(),
            }));
        }

        Ok(None)
    }
}

