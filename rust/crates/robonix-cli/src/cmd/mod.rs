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
mod install;
mod launch_helpers;
mod list;
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
    /// Build a package (local path or system-installed)
    Build {
        /// Local package path (relative to $RBNX_INVOCATION_CWD, else process cwd)
        #[arg(short = 'p', long)]
        path: Option<PathBuf>,
        /// Build by system-installed package name
        #[arg(short = 'g', long)]
        global: Option<String>,
        /// Clean build (remove rbnx-build before building). Default: incremental.
        #[arg(long)]
        clean: bool,
    },
    /// Start the package (runs its single top-level `start` shell block).
    /// Defaults to the package containing the current directory when `-p`
    /// is omitted. Blocks until the process exits. The pre-dev-packaging
    /// `-n / --node` flag is gone — one package = one start body now.
    Start {
        /// Package path or installed name; relative paths use $RBNX_INVOCATION_CWD, else process cwd.
        /// If omitted, rbnx walks up from the current directory to find a package manifest.
        #[arg(short = 'p', long)]
        package: Option<String>,
        /// Registry endpoint (default: 127.0.0.1:50051)
        #[arg(long)]
        endpoint: Option<String>,
    },
    /// Bring up an entire deployment described by a top-level `robonix_manifest.yaml`
    /// — system services (atlas/executor/pilot/liaison/memory/vlm) plus every package
    /// declared under `primitive`/`service`/`skill`. Blocks until Ctrl-C.
    Deploy {
        /// Path to the deployment manifest (default: `./robonix_manifest.yaml`).
        #[arg(short = 'f', long, default_value = "robonix_manifest.yaml")]
        file: PathBuf,
        /// Directory for per-component logs (default: `<manifest-dir>/rbnx-deploy/logs`).
        #[arg(long)]
        log_dir: Option<PathBuf>,
        /// Skip starting the `system:` block (atlas/pilot/etc). Useful when
        /// those are already running externally.
        #[arg(long)]
        skip_system: bool,
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
    /// Validate a package manifest without building. If no path is given,
    /// rbnx walks up from the current directory to find a package manifest.
    Validate {
        /// Package directory (relative paths use $RBNX_INVOCATION_CWD, else process cwd)
        path: Option<PathBuf>,
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
    /// Stages system protos under `<pkg>/rbnx-build/proto-staging/`, then emits
    /// `<pkg>/proto_gen/` (and optional `<pkg>/robonix_mcp_types/`). Replaces the
    /// copy-pasted boilerplate in package build.sh scripts. If `-p` is omitted,
    /// rbnx walks up from the current directory to find a package manifest.
    Codegen {
        /// Package path (relative to $RBNX_INVOCATION_CWD, else process cwd)
        #[arg(short = 'p', long)]
        package: Option<PathBuf>,
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
    /// Keys: root, rust, contracts, interfaces-lib, runtime-proto, robonix-py
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
}

pub async fn execute(command: Commands, config: Config) -> Result<()> {
    match command {
        Commands::Build {
            path,
            global,
            clean,
        } => run_package::execute_build(config, path, global, clean).await,
        Commands::Start {
            package,
            endpoint,
        } => run_package::execute_start(&config, package.as_deref(), endpoint.as_deref()).await,
        Commands::Deploy {
            file,
            log_dir,
            skip_system,
        } => deploy::execute(config, file, log_dir, skip_system).await,
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
    }
}
