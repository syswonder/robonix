// SPDX-License-Identifier: MulanPSL-2.0
// Helpers for `rbnx start` (env escaping for shell fragments).

pub fn shell_escape(value: &str) -> String {
    format!("'{}'", value.replace('\'', "'\"'\"'"))
}
