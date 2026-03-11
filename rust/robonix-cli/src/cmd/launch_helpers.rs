// SPDX-License-Identifier: MulanPSL-2.0
// Helpers for building package start command (used by run_package).

use std::collections::HashMap;
use std::path::Path;

pub fn shell_escape(value: &str) -> String {
    format!("'{}'", value.replace('\'', "'\"'\"'"))
}

pub fn build_start_command(
    install_setup: &Path,
    module: &str,
    env: &HashMap<String, String>,
) -> String {
    let distro = std::env::var("ROS_DISTRO").unwrap_or_else(|_| "humble".to_string());
    let mut command = format!(
        "unset PYTHONNOUSERSITE; set +u; source /opt/ros/{distro}/setup.bash; source {install}; set -u",
        distro = distro,
        install = shell_escape(&install_setup.display().to_string()),
    );
    for (key, value) in env {
        command.push_str(&format!(
            "; export {key}={value}",
            key = key,
            value = shell_escape(value)
        ));
    }
    command.push_str(&format!("; python3 -m {}", module));
    command
}
