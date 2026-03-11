// SPDX-License-Identifier: MulanPSL-2.0
// Command Module
//
// Command definitions and execution for robonix-cli

use anyhow::Result;
use clap::Subcommand;
use std::path::PathBuf;

use crate::Config;

mod build;
mod config;
mod info;
mod install;
mod launch_helpers;
mod list;
mod run_package;
mod search;
mod validate;

#[derive(Subcommand)]
pub enum Commands {
    /// Build a package (requires -p)
    Build {
        /// Package name or path (e.g. python_ping_client or examples/python_ping_client)
        #[arg(short = 'p', long, required = true)]
        package: String,
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
    /// Package management commands
    #[command(subcommand)]
    Package(PackageCommands),
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

#[derive(Subcommand)]
pub enum PackageCommands {
    /// Install a package from GitHub or local path
    Install {
        /// GitHub repository URL (e.g., https://github.com/user/repo.git)
        #[arg(short, long)]
        github: Option<String>,
        /// Branch to clone (for GitHub)
        #[arg(short, long)]
        branch: Option<String>,
        /// Local path to package directory
        #[arg(short, long)]
        path: Option<PathBuf>,
    },
    /// List all installed packages
    List,
    /// Show detailed information about a package
    Info {
        /// Package name
        name: String,
    },
    /// Search packages by capability
    SearchCap {
        /// Capability name (e.g., cap::vision.capture_rgb)
        name: String,
    },
    /// Search packages by skill
    SearchSkill {
        /// Skill name (e.g., skl::pick)
        name: String,
    },
    /// Build a package (compile, install dependencies, etc.)
    ///
    /// Builds a package directly without requiring an active recipe.
    /// Can build a specific package by name or all installed packages.
    Build {
        /// Target to build. Can be:
        /// - "all" to build all installed packages
        /// - Package name (e.g., "demo_rgb_provider")
        #[arg(required = false, default_value = "all")]
        target: String,
    },
    /// Build a local vNext package into package-local rbnx-build workspace
    BuildLocal {
        /// Local path to package directory
        path: PathBuf,
    },
    /// Validate a package manifest without installing it
    Validate {
        /// Local path to package directory
        path: PathBuf,
    },
}

pub async fn execute(command: Commands, config: Config) -> Result<()> {
    match command {
        Commands::Build { package } => run_package::execute_build(&package).await,
        Commands::Start {
            package,
            node,
            endpoint,
        } => run_package::execute_start(&package, &node, endpoint.as_deref()).await,
        Commands::Package(cmd) => match cmd {
            PackageCommands::Install {
                github,
                branch,
                path,
            } => install::execute(config, github, path, branch).await,
            PackageCommands::List => list::execute(config).await,
            PackageCommands::Info { name } => info::execute(config, name).await,
            PackageCommands::SearchCap { name } => search::execute_cap(config, name).await,
            PackageCommands::SearchSkill { name } => search::execute_skill(config, name).await,
            PackageCommands::Build { target } => build::execute_package(config, target).await,
            PackageCommands::BuildLocal { path } => build::execute_local(path).await,
            PackageCommands::Validate { path } => validate::execute(path).await,
        },
        Commands::Config {
            set_storage_path,
            show,
        } => config::execute(config, set_storage_path, show).await,
    }
}
