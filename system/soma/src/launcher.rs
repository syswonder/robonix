// SPDX-License-Identifier: MulanPSL-2.0

use crate::deployment::PackageLaunchTarget;
use anyhow::{Context, Result};
use std::process::Stdio;
use tokio::process::{Child, Command};

#[derive(Debug, Clone)]
pub struct PackageLauncher {
    rbnx_bin: String,
    atlas_endpoint: String,
}

#[derive(Debug)]
pub struct LaunchHandle {
    pub child: Child,
}

impl PackageLauncher {
    /// Build a launcher that starts packages with `rbnx start -p <package_dir>`.
    pub fn new(rbnx_bin: impl Into<String>, atlas_endpoint: impl Into<String>) -> Self {
        Self {
            rbnx_bin: rbnx_bin.into(),
            atlas_endpoint: atlas_endpoint.into(),
        }
    }

    /// Spawn one primitive/skill package and return immediately with the child handle.
    pub fn spawn(&self, target: &PackageLaunchTarget) -> Result<LaunchHandle> {
        let mut cmd = self.command(target);
        let child = cmd.spawn().with_context(|| {
            format!(
                "spawn `{}` for {} '{}'",
                self.command_line(target),
                target.kind,
                target.name
            )
        })?;
        Ok(LaunchHandle { child })
    }

    pub fn command_line(&self, target: &PackageLaunchTarget) -> String {
        format!(
            "{} start -p {} --endpoint {}",
            self.rbnx_bin,
            target.package_dir.display(),
            self.atlas_endpoint
        )
    }

    /// Build the `rbnx start` command with an absolute package path.
    fn command(&self, target: &PackageLaunchTarget) -> Command {
        let mut cmd = Command::new(&self.rbnx_bin);
        cmd.arg("start")
            .arg("-p")
            .arg(target.package_dir.as_os_str())
            .arg("--endpoint")
            .arg(&self.atlas_endpoint)
            .stdin(Stdio::null())
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit());
        cmd
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deployment::{PackageKind, PackageLaunchTarget};
    use std::ffi::OsString;
    use std::path::PathBuf;

    #[test]
    /// The launcher passes `-p` directly and avoids rebasing it through `RBNX_INVOCATION_CWD`.
    fn launch_command_uses_package_path_without_invocation_cwd_override() {
        let target = PackageLaunchTarget {
            kind: PackageKind::Primitive,
            name: "demo".into(),
            package_dir: PathBuf::from("/tmp/robonix/examples/test_ci/primitives/demo"),
            package_manifest_path: PathBuf::from(
                "/tmp/robonix/examples/test_ci/primitives/demo/package_manifest.yaml",
            ),
        };
        let launcher = PackageLauncher::new("rbnx", "127.0.0.1:50051");
        let cmd = launcher.command(&target);
        let args = cmd
            .as_std()
            .get_args()
            .map(OsString::from)
            .collect::<Vec<_>>();

        assert_eq!(
            args,
            vec![
                OsString::from("start"),
                OsString::from("-p"),
                target.package_dir.clone().into_os_string(),
                OsString::from("--endpoint"),
                OsString::from("127.0.0.1:50051"),
            ]
        );
        assert!(
            !cmd.as_std()
                .get_envs()
                .any(|(key, _)| key == "RBNX_INVOCATION_CWD")
        );
    }
}
