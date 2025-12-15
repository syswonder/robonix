use anyhow::Result;
use clap::Subcommand;
use std::path::PathBuf;

use crate::Config;

mod build;
mod config;
mod daemon;
mod info;
mod install;
mod list;
mod recipe_utils;
mod register;
mod restart;
mod search;
mod start;
mod status;
mod stop;
mod task;
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
        /// Set robonix-sdk path
        #[arg(short = 'm', long)]
        set_sdk_path: Option<PathBuf>,
        /// Show current configuration
        #[arg(short, long)]
        show: bool,
    },
    /// Daemon management commands
    #[command(subcommand)]
    Daemon(DaemonCommands),
    /// Task management commands
    #[command(subcommand)]
    Task(TaskCommands),
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
    /// Restart capability or skill processes from active recipe
    /// 
    /// This is equivalent to: stop -> wait -> start
    /// Stops the target processes (if running) and then starts them again.
    Restart {
        /// Target to restart. Can be:
        /// - "all" to restart all processes from recipe
        /// - Pattern like "cap::vision.*" or "*.pick" to match multiple
        /// - Exact name like "cap::vision.capture_rgb"
        #[arg(required = false, default_value = "all")]
        target: String,
    },
    /// Show status of all running cap/skill processes from active recipe
    Status,
    /// Build packages (compile, install dependencies, etc.)
    /// 
    /// Builds packages before deployment. Can build all packages in recipe
    /// or a specific package by name.
    Build {
        /// Target to build. Can be:
        /// - "all" to build all packages in recipe
        /// - Package name (e.g., "demo_rgb_provider")
        #[arg(required = false, default_value = "all")]
        target: String,
    },
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

#[derive(Subcommand)]
pub enum DaemonCommands {
    /// Start the daemon
    Start,
    /// Stop the daemon
    Stop,
    /// Show daemon status
    Status,
    /// Restart the daemon
    Restart,
}

#[derive(Subcommand)]
pub enum TaskCommands {
    /// Create a new task from natural language
    Create {
        /// Natural language task description
        natural_language: String,
    },
    /// Get task by ID
    Get {
        /// Task ID
        task_id: String,
    },
    /// List all tasks
    List,
    /// Cancel a task
    Cancel {
        /// Task ID
        task_id: String,
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
            PackageCommands::Build { target } => build::execute_package(config, target).await,
        },
        Commands::Deploy(cmd) => match cmd {
            DeployCommands::Register { recipe } => register::execute(config, recipe).await,
            DeployCommands::Build { target } => build::execute(config, target).await,
            DeployCommands::Start { target } => start::execute(config, target).await,
            DeployCommands::Stop { target } => stop::execute(config, target).await,
            DeployCommands::Restart { target } => restart::execute(config, target).await,
            DeployCommands::Status => status::execute(config).await,
            DeployCommands::Unregister { target } => unregister::execute(config, target).await,
        },
        Commands::Config {
            set_storage_path,
            set_sdk_path,
            show,
        } => config::execute(config, set_storage_path, set_sdk_path, show).await,
        Commands::Daemon(cmd) => match cmd {
            DaemonCommands::Start => daemon::start().await,
            DaemonCommands::Stop => daemon::stop().await,
            DaemonCommands::Status => daemon::status().await,
            DaemonCommands::Restart => daemon::restart().await,
        },
        Commands::Task(cmd) => match cmd {
            TaskCommands::Create { natural_language } => {
                task::execute_create(config, natural_language).await
            }
            TaskCommands::Get { task_id } => task::execute_get(config, task_id).await,
            TaskCommands::List => task::execute_list(config).await,
            TaskCommands::Cancel { task_id } => task::execute_cancel(config, task_id).await,
        },
    }
}
