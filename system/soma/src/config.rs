// SPDX-License-Identifier: MulanPSL-2.0

use anyhow::{Context, Result, bail};
use clap::Parser;
use serde::Deserialize;
use std::path::{Component, Path, PathBuf};

pub const DEFAULT_PROVIDER_ID: &str = "soma";
pub const DEFAULT_ATLAS_ENDPOINT: &str = "127.0.0.1:50051";
pub const DEFAULT_LISTEN: &str = "127.0.0.1:50091";

#[derive(Debug, Clone)]
pub struct SomaConfig {
    pub atlas_endpoint: String,
    pub listen: String,
    pub provider_id: String,
    pub robonix_root: PathBuf,
    pub default_robot: Option<String>,
    pub deployments: Vec<DeploymentConfig>,
    pub start_packages: bool,
    pub rbnx_bin: String,
}

#[derive(Debug, Clone)]
pub struct DeploymentConfig {
    pub path: PathBuf,
}

#[derive(Parser, Debug)]
#[command(name = "robonix-soma", about = "Robonix Soma raw body service")]
pub struct Args {
    #[arg(long, env = "ROBONIX_ATLAS_ENDPOINT")]
    pub atlas: Option<String>,

    #[arg(long, env = "ROBONIX_SOMA_LISTEN")]
    pub listen: Option<String>,

    #[arg(long, env = "ROBONIX_SOMA_PROVIDER_ID")]
    pub provider_id: Option<String>,

    #[arg(long, env = "ROBONIX_SOMA_DEFAULT_ROBOT")]
    pub default_robot: Option<String>,

    /// Robonix source root used as the base for relative deployment paths.
    #[arg(long, env = "ROBONIX_SOMA_ROBONIX_ROOT")]
    pub robonix_root: Option<PathBuf>,

    /// Deployment directory containing robonix_manifest.yaml. May repeat.
    #[arg(
        long = "deployment",
        env = "ROBONIX_SOMA_DEPLOYMENTS",
        value_delimiter = ','
    )]
    pub deployments: Vec<PathBuf>,

    #[arg(long, env = "ROBONIX_CONFIG_PATH")]
    pub config: Option<PathBuf>,

    #[arg(long, env = "ROBONIX_SOMA_RBNX_BIN")]
    pub rbnx_bin: Option<String>,

    /// Log level for this component (`debug`/`info`/`warn`/`error`). Sets the
    /// scribe log-file floor; falls back to `SCRIBE_FILE_LEVEL` / `info`.
    /// Normally arrives inside `--config-json`, not as a standalone flag.
    #[arg(long)]
    pub log: Option<String>,

    /// The component's `system.soma` manifest block, serialized to JSON by
    /// rbnx and passed as one arg (`--config-json '{…}'`). Parsed by the binary
    /// itself — see `robonix_scribe::init_from_config`, which reads the `log`
    /// key from it so the manifest's per-component level reaches the log.
    #[arg(long)]
    pub config_json: Option<String>,
}

#[derive(Default, Deserialize)]
struct FileConfig {
    #[serde(default)]
    atlas_endpoint: Option<String>,
    #[serde(default)]
    listen: Option<String>,
    #[serde(default)]
    provider_id: Option<String>,
    #[serde(default)]
    default_robot: Option<String>,
    #[serde(default)]
    robonix_root: Option<PathBuf>,
    #[serde(default)]
    deployments: Vec<DeploymentEntry>,
    #[serde(default)]
    start_packages: Option<bool>,
    #[serde(default)]
    rbnx_bin: Option<String>,
}

#[derive(Clone, Deserialize)]
#[serde(untagged)]
enum DeploymentEntry {
    Path(PathBuf),
    Object { path: PathBuf },
}

impl SomaConfig {
    /// Resolve final config from defaults, optional config file, env, and CLI args.
    pub fn resolve(args: Args) -> Result<Self> {
        let file_cfg = match &args.config {
            Some(path) => load_yaml(path)?,
            None => FileConfig::default(),
        };

        let config_dir = args.config.as_deref().and_then(Path::parent);
        let robonix_root = resolve_robonix_root(
            args.robonix_root.clone(),
            file_cfg.robonix_root.clone(),
            config_dir,
        )?;
        let mut deployments = if args.deployments.is_empty() {
            file_cfg
                .deployments
                .into_iter()
                .map(|entry| match entry {
                    DeploymentEntry::Path(path) | DeploymentEntry::Object { path } => {
                        DeploymentConfig { path }
                    }
                })
                .collect::<Vec<_>>()
        } else {
            args.deployments
                .into_iter()
                .map(|path| DeploymentConfig { path })
                .collect::<Vec<_>>()
        };
        normalize_deployments(&mut deployments, &robonix_root);
        if deployments.is_empty() {
            bail!("missing Soma deployments: set --deployment or config deployments");
        }

        Ok(Self {
            atlas_endpoint: args
                .atlas
                .or(file_cfg.atlas_endpoint)
                .unwrap_or_else(|| DEFAULT_ATLAS_ENDPOINT.to_string()),
            listen: args
                .listen
                .or(file_cfg.listen)
                .unwrap_or_else(|| DEFAULT_LISTEN.to_string()),
            provider_id: args
                .provider_id
                .or(file_cfg.provider_id)
                .unwrap_or_else(|| DEFAULT_PROVIDER_ID.to_string()),
            robonix_root,
            default_robot: args.default_robot.or(file_cfg.default_robot),
            deployments,
            start_packages: file_cfg.start_packages.unwrap_or(false),
            rbnx_bin: args
                .rbnx_bin
                .or(file_cfg.rbnx_bin)
                .unwrap_or_else(|| "rbnx".into()),
        })
    }
}

fn load_yaml(path: &Path) -> Result<FileConfig> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("read Soma config '{}'", path.display()))?;
    serde_yaml::from_str(&raw).with_context(|| format!("parse Soma config '{}'", path.display()))
}

/// Resolve relative deployment paths against the Robonix source root.
fn normalize_deployments(deployments: &mut [DeploymentConfig], robonix_root: &Path) {
    for deployment in deployments {
        if deployment.path.is_relative() {
            deployment.path = normalize_path(robonix_root.join(&deployment.path));
        }
    }
}

fn resolve_robonix_root(
    cli_root: Option<PathBuf>,
    file_root: Option<PathBuf>,
    config_dir: Option<&Path>,
) -> Result<PathBuf> {
    if let Some(root) = cli_root {
        return absolute_root(root, None);
    }
    if let Some(root) = file_root {
        return absolute_root(root, config_dir);
    }
    for key in ["ROBONIX_SOURCE_PATH", "ROBONIX_ROOT"] {
        if let Ok(value) = std::env::var(key)
            && !value.trim().is_empty()
        {
            return absolute_root(PathBuf::from(value), None);
        }
    }
    find_robonix_root_from_cwd()
}

/// Resolve a Robonix root to an absolute syntactic path without requiring it to exist.
fn absolute_root(root: PathBuf, relative_base: Option<&Path>) -> Result<PathBuf> {
    if root.is_absolute() {
        Ok(normalize_path(root))
    } else {
        let base = match relative_base {
            Some(base) if base.is_absolute() => base.to_path_buf(),
            Some(base) => std::env::current_dir()
                .context("resolve current directory")?
                .join(base),
            None => std::env::current_dir().context("resolve current directory")?,
        };
        Ok(normalize_path(base.join(root)))
    }
}

/// Normalize path syntax without touching the filesystem; a lone `.` remains `.`.
fn normalize_path(path: PathBuf) -> PathBuf {
    let mut out = PathBuf::new();
    for component in path.components() {
        match component {
            Component::CurDir => {}
            Component::ParentDir => {
                out.pop();
            }
            other => out.push(other.as_os_str()),
        }
    }
    if out.as_os_str().is_empty() {
        out.push(".");
    }
    out
}

fn find_robonix_root_from_cwd() -> Result<PathBuf> {
    let start = std::env::current_dir().context("resolve current directory")?;
    for candidate in start.ancestors() {
        if candidate.join("Cargo.toml").is_file() && candidate.join("capabilities").is_dir() {
            return Ok(candidate.to_path_buf());
        }
    }
    bail!(
        "could not locate Robonix root from '{}'; set --robonix-root, robonix_root, ROBONIX_SOURCE_PATH, or ROBONIX_ROOT",
        start.display()
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolves_cli_deployment_paths() {
        let args = Args {
            atlas: None,
            listen: None,
            provider_id: None,
            default_robot: Some("demo".into()),
            robonix_root: Some(repo_root()),
            deployments: vec![PathBuf::from("examples/test_ci")],
            config: None,
            rbnx_bin: None,
            log: None,
            config_json: None,
        };
        let cfg = SomaConfig::resolve(args).expect("resolve config");
        assert_eq!(cfg.default_robot.as_deref(), Some("demo"));
        assert_eq!(cfg.deployments.len(), 1);
        assert!(cfg.deployments[0].path.is_absolute());
        assert!(cfg.deployments[0].path.ends_with("examples/test_ci"));
    }

    #[test]
    fn config_file_relative_deployments_use_robonix_root() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let config_dir = tmp.path().join("system/soma");
        std::fs::create_dir_all(&config_dir).expect("create config dir");
        let config_path = config_dir.join("soma.yaml");
        std::fs::write(
            &config_path,
            "robonix_root: ../..\ndeployments:\n  - examples/test_ci\nstart_packages: false\n",
        )
        .expect("write config");
        let args = Args {
            atlas: None,
            listen: None,
            provider_id: None,
            default_robot: None,
            robonix_root: None,
            deployments: vec![],
            config: Some(config_path),
            rbnx_bin: None,
            log: None,
            config_json: None,
        };
        let cfg = SomaConfig::resolve(args).expect("resolve config");
        assert_eq!(cfg.deployments.len(), 1);
        assert_eq!(cfg.deployments[0].path, tmp.path().join("examples/test_ci"));
        assert!(!cfg.start_packages);
    }

    #[test]
    fn dot_robonix_root_and_dot_deployment_resolve_to_config_dir() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let config_path = tmp.path().join("config.yaml");
        std::fs::write(
            &config_path,
            "robonix_root: .\ndeployments:\n  - .\nstart_packages: false\n",
        )
        .expect("write config");
        let args = Args {
            atlas: None,
            listen: None,
            provider_id: None,
            default_robot: None,
            robonix_root: None,
            deployments: vec![],
            config: Some(config_path),
            rbnx_bin: None,
            log: None,
            config_json: None,
        };
        let cfg = SomaConfig::resolve(args).expect("resolve config");
        assert_eq!(cfg.robonix_root, tmp.path());
        assert_eq!(cfg.deployments[0].path, tmp.path());
        assert!(!cfg.start_packages);
    }

    #[test]
    fn start_packages_defaults_to_false_when_config_omits_it() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let config_path = tmp.path().join("config.yaml");
        std::fs::write(&config_path, "robonix_root: .\ndeployments:\n  - .\n")
            .expect("write config");
        let args = Args {
            atlas: None,
            listen: None,
            provider_id: None,
            default_robot: None,
            robonix_root: None,
            deployments: vec![],
            config: Some(config_path),
            rbnx_bin: None,
            log: None,
            config_json: None,
        };
        let cfg = SomaConfig::resolve(args).expect("resolve config");
        assert!(!cfg.start_packages);
    }

    #[test]
    fn cli_rbnx_bin_overrides_config_file() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let config_path = tmp.path().join("config.yaml");
        std::fs::write(
            &config_path,
            "robonix_root: .\ndeployments:\n  - .\nrbnx_bin: file-rbnx\n",
        )
        .expect("write config");
        let args = Args {
            atlas: None,
            listen: None,
            provider_id: None,
            default_robot: None,
            robonix_root: None,
            deployments: vec![],
            config: Some(config_path),
            rbnx_bin: Some("cli-rbnx".into()),
            log: None,
            config_json: None,
        };
        let cfg = SomaConfig::resolve(args).expect("resolve config");

        assert_eq!(cfg.rbnx_bin, "cli-rbnx");
    }

    #[test]
    fn rbnx_bin_defaults_when_unset() {
        let args = Args {
            atlas: None,
            listen: None,
            provider_id: None,
            default_robot: None,
            robonix_root: Some(repo_root()),
            deployments: vec![PathBuf::from("examples/test_ci")],
            config: None,
            rbnx_bin: None,
            log: None,
            config_json: None,
        };
        let cfg = SomaConfig::resolve(args).expect("resolve config");

        assert_eq!(cfg.rbnx_bin, "rbnx");
    }

    #[test]
    fn normalize_path_keeps_single_dot() {
        assert_eq!(normalize_path(PathBuf::from(".")), PathBuf::from("."));
    }

    #[test]
    /// Relative config directories are anchored to the process cwd before root resolution.
    fn relative_config_dir_resolves_root_to_absolute_path() {
        let expected = normalize_path(
            std::env::current_dir()
                .expect("current dir")
                .join("system/soma")
                .join("../.."),
        );
        let actual =
            absolute_root(PathBuf::from("../.."), Some(Path::new("system/soma"))).expect("root");

        assert!(actual.is_absolute());
        assert_eq!(actual, expected);
    }

    #[test]
    /// A relative `--config` path must still produce absolute deployment paths.
    fn relative_config_path_resolves_deployments_to_absolute_paths() {
        let cwd = std::env::current_dir().expect("current dir");
        let tmp = tempfile::Builder::new()
            .prefix("soma-relative-config-")
            .tempdir_in(&cwd)
            .expect("tempdir");
        let config_dir = tmp.path().join("system/soma");
        std::fs::create_dir_all(&config_dir).expect("create config dir");
        let config_path = config_dir.join("config.yaml");
        std::fs::write(
            &config_path,
            "robonix_root: ../..\ndeployments:\n  - examples/test_ci\n",
        )
        .expect("write config");
        let relative_config_path = config_path
            .strip_prefix(&cwd)
            .expect("relative config path")
            .to_path_buf();
        let args = Args {
            atlas: None,
            listen: None,
            provider_id: None,
            default_robot: None,
            robonix_root: None,
            deployments: vec![],
            config: Some(relative_config_path),
            rbnx_bin: None,
            log: None,
            config_json: None,
        };
        let cfg = SomaConfig::resolve(args).expect("resolve config");

        assert_eq!(cfg.robonix_root, tmp.path());
        assert_eq!(cfg.deployments[0].path, tmp.path().join("examples/test_ci"));
        assert!(cfg.deployments[0].path.is_absolute());
    }

    #[test]
    fn bundled_config_resolves_from_config_location() {
        let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let args = Args {
            atlas: None,
            listen: None,
            provider_id: None,
            default_robot: None,
            robonix_root: None,
            deployments: vec![],
            config: Some(manifest_dir.join("config.yaml")),
            rbnx_bin: None,
            log: None,
            config_json: None,
        };
        let cfg = SomaConfig::resolve(args).expect("resolve config");
        assert_eq!(cfg.robonix_root, repo_root());
        assert_eq!(
            cfg.deployments[0].path,
            repo_root().join("examples/test_ci")
        );
    }

    fn repo_root() -> PathBuf {
        normalize_path(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../.."))
    }
}
