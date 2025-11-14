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
    /// Register packages from a recipe file (only registers, does not start)
    Register {
        /// Recipe file path
        recipe: PathBuf,
    },
    /// Unregister packages, capabilities, skills, or recipes (only unregisters, does not stop)
    Unregister {
        /// Target to unregister. Can be:
        /// - package name (e.g., "demo_rgb_provider")
        /// - package.capability (e.g., "demo_rgb_provider.cap::vision.capture_rgb")
        /// - package.skill (e.g., "demo_rgb_provider.skl::pick")
        /// - recipe file path (e.g., "demo_recipe.yaml")
        target: String,
    },
    /// Show status of all running cap/skill processes
    Status,
    /// Start a capability or skill process from active recipe
    Start {
        /// Target to start. Can be:
        /// - "all" to start all items in recipe
        /// - Pattern like "cap::vision.*" or "*.pick" to match multiple
        /// - Exact name like "cap::vision.capture_rgb"
        #[arg(required = false, default_value = "all")]
        target: String,
    },
    /// Stop a capability or skill process from active recipe
    Stop {
        /// Target to stop. Can be:
        /// - "all" to stop all running processes from recipe
        /// - Pattern like "cap::vision.*" or "*.pick" to match multiple
        /// - Exact name like "cap::vision.capture_rgb"
        #[arg(required = false, default_value = "all")]
        target: String,
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
        Commands::Install {
            github,
            branch,
            path,
        } => install::execute(config, github, path, branch).await,
        Commands::List => list::execute(config).await,
        Commands::Info { name } => info::execute(config, name).await,
        Commands::SearchCap { name } => search::execute_cap(config, name).await,
        Commands::SearchSkill { name } => search::execute_skill(config, name).await,
        Commands::Register { recipe } => register::execute(config, recipe).await,
        Commands::Unregister { target } => unregister::execute(config, target).await,
        Commands::Status => status::execute(config).await,
        Commands::Start { target } => start::execute(config, target).await,
        Commands::Stop { target } => stop::execute(config, target).await,
        Commands::Config {
            set_storage_path,
            show,
        } => config::execute(config, set_storage_path, show).await,
    }
}
