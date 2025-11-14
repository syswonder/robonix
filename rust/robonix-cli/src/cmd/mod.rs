use anyhow::Result;
use clap::{arg, Subcommand};
use std::path::PathBuf;

use crate::Config;

mod config;
mod info;
mod install;
mod list;
mod recipe_utils;
mod register;
mod search;
mod start;
mod status;
mod stop;
mod unregister;

#[derive(Subcommand)]
pub enum Commands {
    /// Package management commands
    #[command(subcommand)]
    Package(PackageCommands),
    /// Deploy and manage recipes (workflow: register -> start -> stop -> unregister)
    #[command(subcommand)]
    Deploy(DeployCommands),
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
}

#[derive(Subcommand)]
pub enum DeployCommands {
    /// Register packages from a recipe file (only registers, does not start)
    /// 
    /// Workflow order: register -> start -> stop -> unregister
    /// You must register before starting processes.
    Register {
        /// Recipe file path
        recipe: PathBuf,
    },
    /// Start capability or skill processes from active recipe
    /// 
    /// Requires: recipe must be registered first (use 'deploy register')
    /// You can start/stop multiple times during deployment.
    Start {
        /// Target to start. Can be:
        /// - "all" to start all items in recipe
        /// - Pattern like "cap::vision.*" or "*.pick" to match multiple
        /// - Exact name like "cap::vision.capture_rgb"
        #[arg(required = false, default_value = "all")]
        target: String,
    },
    /// Stop capability or skill processes from active recipe
    /// 
    /// Requires: processes must be running (use 'deploy start' first)
    /// You can start/stop multiple times during deployment.
    Stop {
        /// Target to stop. Can be:
        /// - "all" to stop all running processes from recipe
        /// - Pattern like "cap::vision.*" or "*.pick" to match multiple
        /// - Exact name like "cap::vision.capture_rgb"
        #[arg(required = false, default_value = "all")]
        target: String,
    },
    /// Show status of all running cap/skill processes from active recipe
    Status,
    /// Unregister packages, capabilities, skills, or recipes
    /// 
    /// Requires: all processes must be stopped first (use 'deploy stop')
    /// This is the final step in the workflow: register -> start -> stop -> unregister
    Unregister {
        /// Target to unregister. Can be:
        /// - package name (e.g., "demo_rgb_provider")
        /// - package.capability (e.g., "demo_rgb_provider.cap::vision.capture_rgb")
        /// - package.skill (e.g., "demo_rgb_provider.skl::pick")
        /// - recipe file path (e.g., "demo_recipe.yaml")
        target: String,
    },
}

pub async fn execute(command: Commands, config: Config) -> Result<()> {
    match command {
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
        },
        Commands::Deploy(cmd) => match cmd {
            DeployCommands::Register { recipe } => register::execute(config, recipe).await,
            DeployCommands::Start { target } => start::execute(config, target).await,
            DeployCommands::Stop { target } => stop::execute(config, target).await,
            DeployCommands::Status => status::execute(config).await,
            DeployCommands::Unregister { target } => unregister::execute(config, target).await,
        },
        Commands::Config {
            set_storage_path,
            show,
        } => config::execute(config, set_storage_path, show).await,
    }
}
