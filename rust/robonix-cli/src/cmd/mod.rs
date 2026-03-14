// SPDX-License-Identifier: MulanPSL-2.0
// Command Module
//
// Command definitions and execution for robonix-cli

use anyhow::Result;
use clap::Subcommand;
use std::path::PathBuf;

use robonix_cli::Config;

mod build;
mod config;
mod info;
mod install;
mod launch_helpers;
mod list;
mod run_package;
mod validate;

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
}

pub async fn execute(command: Commands, config: Config) -> Result<()> {
    match command {
        Commands::Build { path, global } => run_package::execute_build(config, path, global).await,
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
    }
}
