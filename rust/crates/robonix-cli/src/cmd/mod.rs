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
mod config;
mod graph;
mod info;
mod install;
mod launch_helpers;
mod list;
mod run_package;
mod runtime;
mod validate;

const DEFAULT_ENDPOINT: &str = "localhost:50051";

#[derive(Args)]
pub struct GraphArgs {
    /// robonix-server endpoint (ignored with `--test`)
    #[arg(long, env = "ROBONIX_SERVER", default_value = DEFAULT_ENDPOINT)]
    pub server: String,
    /// Output file path (.png or .svg selects format)
    #[arg(short, long, default_value = "topology.png")]
    pub output: PathBuf,
    /// Random mock nodes and channels only (no server)
    #[arg(long = "test")]
    pub test_mode: bool,
}

#[derive(Subcommand)]
pub enum Commands {
    /// Build a package (local path or system-installed)
    Build {
        /// Build by local path (e.g. examples/skill_demo)
        #[arg(short = 'p', long)]
        path: Option<PathBuf>,
        /// Build by system-installed package name
        #[arg(short = 'g', long)]
        global: Option<String>,
        /// Clean build (remove rbnx-build before building). Default: incremental.
        #[arg(long)]
        clean: bool,
    },
    /// Start one node of a package (package and node required). Blocks until the process exits.
    Start {
        /// Package name or path
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
        /// Local path to package directory
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

    /// List all registered nodes and their interfaces
    Nodes {
        /// robonix-server endpoint
        #[arg(long, env = "ROBONIX_SERVER", default_value = DEFAULT_ENDPOINT)]
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
        /// robonix-server endpoint
        #[arg(long, env = "ROBONIX_SERVER", default_value = DEFAULT_ENDPOINT)]
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
        /// robonix-server endpoint
        #[arg(long, env = "ROBONIX_SERVER", default_value = DEFAULT_ENDPOINT)]
        server: String,
        /// Output as JSON
        #[arg(long)]
        json: bool,
    },
    /// Show active channels (negotiated connections)
    Channels {
        /// robonix-server endpoint
        #[arg(long, env = "ROBONIX_SERVER", default_value = DEFAULT_ENDPOINT)]
        server: String,
    },
    /// Dump full runtime state as JSON (nodes, interfaces, channels)
    Inspect {
        /// robonix-server endpoint
        #[arg(long, env = "ROBONIX_SERVER", default_value = DEFAULT_ENDPOINT)]
        server: String,
    },

    /// Chat with the Robonix agent in an interactive TUI
    Chat {
        /// robonix-server endpoint (used to discover agent)
        #[arg(long, env = "ROBONIX_SERVER", default_value = DEFAULT_ENDPOINT)]
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
        Commands::Graph { args } => graph::execute(&args.server, args.output, args.test_mode).await,
    }
}
