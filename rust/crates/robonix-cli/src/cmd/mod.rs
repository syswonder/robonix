// SPDX-License-Identifier: MulanPSL-2.0
// Command Module
//
// Command definitions and execution for robonix-cli

use anyhow::Result;
use clap::{Args, Subcommand};
use std::path::PathBuf;

use robonix_cli::Config;

mod build;
mod chat;
mod codegen;
mod config;
mod deploy;
mod graph;
mod info;
mod init;
mod install;
mod launch_helpers;
mod list;
mod package_new;
mod path;
mod run_package;
mod runtime;
mod setup;
mod validate;

const DEFAULT_ENDPOINT: &str = "localhost:50051";

#[derive(Args)]
pub struct GraphArgs {
    /// robonix-atlas endpoint (ignored with `--test`)
    #[arg(long, env = "ROBONIX_ATLAS", default_value = DEFAULT_ENDPOINT)]
    pub server: String,
    /// Output file path (extension hints format when `--format` omitted: `.png` / `.svg`)
    #[arg(short, long, default_value = "topology.png")]
    pub output: PathBuf,
    /// Output format (`png` default when extension is absent or unrecognized)
    #[arg(long, value_enum)]
    pub format: Option<graph::GraphOutputFormat>,
    /// Random mock nodes and channels only (no server)
    #[arg(long = "test")]
    pub test_mode: bool,
}

#[derive(Subcommand)]
pub enum Commands {
    /// Initialize a new robonix workspace
    Init {
        /// Workspace name (creates a directory with this name)
        name: String,
        /// Parent directory where the workspace will be created (default: current directory)
        #[arg(short = 'p', long)]
        path: Option<PathBuf>,
    },
    /// Create a new package under packages/
    #[command(name = "package-new")]
    PackageNew {
        /// Package name
        name: String,
        /// Parent directory where the package will be created (default: current directory)
        #[arg(short = 'p', long)]
        path: Option<PathBuf>,
    },
    /// Build packages: single package, by target, by config, or entire workspace
    Build {
        /// Target name: find matching config in deploy/ directory
        #[arg(value_name = "TARGET", conflicts_with_all = ["path", "global", "config"])]
        target: Option<String>,
        /// Build by local path (e.g. packages/my_pkg)
        #[arg(short = 'p', long, conflicts_with_all = ["target", "config"])]
        path: Option<PathBuf>,
        /// Build by system-installed package name
        #[arg(short = 'g', long, conflicts_with_all = ["target", "config"])]
        global: Option<String>,
        /// Build from a specific config file
        #[arg(short = 'c', long = "config", conflicts_with_all = ["target", "path", "global"])]
        config: Option<PathBuf>,
        /// Clean build (remove rbnx-build before building). Default: incremental.
        #[arg(long)]
        clean: bool,
    },
    /// Start one node of a package (package and node required). Blocks until the process exits.
    Start {
        /// Package path or installed name; relative paths use $RBNX_INVOCATION_CWD, else process cwd
        #[arg(short = 'p', long, required = true)]
        package: String,
        /// Node id to start (e.g. call_ping). Exactly one node per invocation.
        #[arg(short = 'n', long, required = true)]
        node: String,
        /// Registry endpoint (default: 127.0.0.1:50051)
        #[arg(long)]
        endpoint: Option<String>,
    },
    /// Install a package from GitHub or local path
    Install {
        /// Install from GitHub (e.g. user/repo or https://github.com/user/repo)
        #[arg(long)]
        github: Option<String>,
        /// Install from local path
        #[arg(long)]
        path: Option<PathBuf>,
    },
    /// List system-installed packages
    List,
    /// Show details of a system-installed package
    Info {
        /// Package name
        name: String,
    },
    /// Validate a package manifest without building
    Validate {
        /// Package directory (relative paths use $RBNX_INVOCATION_CWD, else process cwd)
        path: PathBuf,
    },
    /// Configure robonix-cli
    Config {
        /// Set package storage path
        #[arg(short = 'p', long)]
        set_storage_path: Option<PathBuf>,
        /// Show current configuration
        #[arg(short, long)]
        show: bool,
    },
    /// Run codegen for a package (wraps robonix-codegen + grpc_tools.protoc).
    /// Regenerates robonix_proto, <pkg>/proto_gen/, and optional <pkg>/robonix_mcp_types/.
    /// Replaces the copy-pasted boilerplate in package build.sh scripts.
    Codegen {
        /// Package path (relative to $RBNX_INVOCATION_CWD, else process cwd)
        #[arg(short = 'p', long, required = true)]
        package: PathBuf,
        /// Also generate robonix_mcp_types/ (for MCP-based packages)
        #[arg(long)]
        mcp: bool,
        /// Remove previous proto_gen/, robonix_mcp_types/, rbnx-build/ before regenerating
        #[arg(long)]
        clean: bool,
        /// Directory (relative to package root, or absolute) where proto_gen/ and robonix_mcp_types/
        /// should be placed. Defaults to package root; use e.g. `--out-dir tiago_bridge` to put
        /// stubs inside a package subdirectory.
        #[arg(long)]
        out_dir: Option<PathBuf>,
    },
    /// Register this directory as the robonix source tree (persists to ~/.robonix/config.yaml).
    /// Call once from a cloned robonix repo so packages anywhere on disk can find contracts/IDL.
    Setup {
        /// Path to the robonix repo root (default: $RBNX_INVOCATION_CWD or process cwd).
        /// If the given path is a sub-directory, walks up to find the root.
        path: Option<PathBuf>,
    },
    /// Print an absolute path rooted in the configured robonix source tree (for build scripts).
    /// Keys: root, rust, contracts, interfaces-lib, interfaces-proto, runtime-proto, robonix-py
    Path {
        /// Path key to resolve (see above).
        key: String,
    },

    /// List all registered nodes and their interfaces
    Nodes {
        /// robonix-atlas endpoint
        #[arg(long, env = "ROBONIX_ATLAS", default_value = DEFAULT_ENDPOINT)]
        server: String,
        /// Filter by distro prefix (e.g. humble)
        #[arg(long)]
        distro: Option<String>,
        /// Filter by exact container_id
        #[arg(long)]
        container: Option<String>,
        /// Output as JSON
        #[arg(long)]
        json: bool,
    },
    /// Show SKILL.md for registered nodes (all, or one with --node)
    Describe {
        /// robonix-atlas endpoint
        #[arg(long, env = "ROBONIX_ATLAS", default_value = DEFAULT_ENDPOINT)]
        server: String,
        /// Show full SKILL.md for a specific node
        #[arg(long)]
        node: Option<String>,
        /// Output as JSON
        #[arg(long)]
        json: bool,
    },
    /// Print all tools visible to the agent (builtin + node skills)
    Tools {
        /// robonix-atlas endpoint
        #[arg(long, env = "ROBONIX_ATLAS", default_value = DEFAULT_ENDPOINT)]
        server: String,
        /// Output as JSON
        #[arg(long)]
        json: bool,
    },
    /// Show active channels (negotiated connections)
    Channels {
        /// robonix-atlas endpoint
        #[arg(long, env = "ROBONIX_ATLAS", default_value = DEFAULT_ENDPOINT)]
        server: String,
    },
    /// Dump full runtime state as JSON (nodes, interfaces, channels)
    Inspect {
        /// robonix-atlas endpoint
        #[arg(long, env = "ROBONIX_ATLAS", default_value = DEFAULT_ENDPOINT)]
        server: String,
    },

    /// Chat with the Robonix agent in an interactive TUI
    Chat {
        /// robonix-atlas endpoint (used to discover agent)
        #[arg(long, env = "ROBONIX_ATLAS", default_value = DEFAULT_ENDPOINT)]
        server: String,
    },

    /// Generate a topology graph of the running system
    Graph {
        #[command(flatten)]
        args: GraphArgs,
    },

    /// Deploy a full stack: by target name, by config file, or from workspace
    Deploy {
        /// Target name: find matching config in deploy/ directory
        #[arg(value_name = "TARGET", conflicts_with = "config")]
        target: Option<String>,
        /// Deploy from a specific config file
        #[arg(short = 'c', long = "config", conflicts_with = "target")]
        config: Option<PathBuf>,
    },
}

pub async fn execute(command: Commands, config: Config) -> Result<()> {
    match command {
        Commands::Init { name, path } => init::execute(&name, path.as_deref()).await,
        Commands::PackageNew { name, path } => package_new::execute(&name, path.as_deref()).await,
        Commands::Build {
            target,
            path,
            global,
            config: config_file,
            clean,
        } => {
            if let Some(p) = path {
                // -p mode: single-package build (preserved)
                build::execute_local(p, clean).await
            } else if let Some(ref g) = global {
                // -g mode: global package build (preserved)
                run_package::execute_build(config, None, Some(g.clone()), clean).await
            } else if let Some(cfg_path) = config_file {
                // -c mode: build from specified config file
                build::execute_from_config(cfg_path, clean).await
            } else if let Some(target_name) = target {
                // <target> mode: find deploy/<target>.yaml
                let cfg_path = find_deploy_config_by_target(&target_name)?;
                build::execute_from_config(cfg_path, clean).await
            } else {
                // No args: workspace-level build from robonix_workspace.yaml
                build::execute_from_workspace(clean).await
            }
        }
        Commands::Start {
            package,
            node,
            endpoint,
        } => run_package::execute_start(&config, &package, &node, endpoint.as_deref()).await,
        Commands::Install { github, path } => install::execute(config, github, path).await,
        Commands::List => list::execute(config).await,
        Commands::Info { name } => info::execute(config, &name).await,
        Commands::Validate { path } => validate::execute(path).await,
        Commands::Config {
            set_storage_path,
            show,
        } => config::execute(config, set_storage_path, show).await,
        Commands::Codegen {
            package,
            mcp,
            clean,
            out_dir,
        } => codegen::execute(config, package, mcp, clean, out_dir).await,
        Commands::Setup { path } => setup::execute(config, path).await,
        Commands::Path { key } => path::execute(config, key).await,
        Commands::Nodes {
            server,
            distro,
            container,
            json,
        } => runtime::nodes(&server, distro.as_deref(), container.as_deref(), json).await,
        Commands::Describe { server, node, json } => {
            runtime::describe(&server, node.as_deref(), json).await
        }
        Commands::Tools { server, json } => runtime::tools(&server, json).await,
        Commands::Channels { server } => runtime::channels(&server).await,
        Commands::Inspect { server } => runtime::inspect(&server).await,
        Commands::Chat { server } => chat::execute(&server).await,
        Commands::Graph { args } => {
            graph::execute(&args.server, args.output, args.format, args.test_mode).await
        }
        Commands::Deploy { target, config } => {
            if let Some(cfg_path) = config {
                let cfg_path = cfg_path.canonicalize().unwrap_or(cfg_path);
                deploy::execute(&cfg_path).await
            } else if let Some(target_name) = target {
                let cfg_path = find_deploy_config_by_target(&target_name)?;
                deploy::execute(&cfg_path).await
            } else {
                // No args: workspace-level deploy from robonix_workspace.yaml
                deploy::execute_from_workspace().await
            }
        }
    }
}

/// Find a deploy config file by target name.
/// Searches `deploy/` directory for `<target>.yaml` or a file with `target: <name>`.
fn find_deploy_config_by_target(target: &str) -> Result<PathBuf> {
    let deploy_dir = PathBuf::from("deploy");
    if !deploy_dir.is_dir() {
        anyhow::bail!(
            "deploy/ directory not found in current directory; \
             use '-c <path>' to specify a config file directly"
        );
    }

    // Strategy 1: direct filename match deploy/<target>.yaml
    let direct = deploy_dir.join(format!("{}.yaml", target));
    if direct.exists() {
        return Ok(direct.canonicalize()?);
    }

    // Strategy 2: scan deploy/*.yaml for matching `target` field
    use serde::Deserialize;
    #[derive(Deserialize)]
    struct Peek {
        target: Option<String>,
    }

    for entry in std::fs::read_dir(&deploy_dir)? {
        let entry = entry?;
        let path = entry.path();
        if path
            .extension()
            .map(|e| e == "yaml" || e == "yml")
            .unwrap_or(false)
        {
            if let Ok(content) = std::fs::read_to_string(&path) {
                if let Ok(peek) = serde_yaml::from_str::<Peek>(&content) {
                    if peek.target.as_deref() == Some(target) {
                        return Ok(path.canonicalize()?);
                    }
                }
            }
        }
    }

    anyhow::bail!(
        "no deploy config found for target '{}' in deploy/ directory.\n\
         Expected deploy/{}.yaml or a file with 'target: {}' field.",
        target,
        target,
        target
    );
}
