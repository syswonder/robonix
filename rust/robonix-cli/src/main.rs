use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use robonix_cli::*;
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "rbnx")]
#[command(about = "Robonix Package Manager CLI", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
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
    /// Register packages from a recipe file
    Register {
        /// Recipe file path
        recipe: PathBuf,
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

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| {
                    tracing_subscriber::EnvFilter::new("info")
                        .add_directive("rustdds=off".parse().unwrap())
                        .add_directive("ros2_client=warn".parse().unwrap())
                }),
        )
        .init();

    let cli = Cli::parse();

    let config = Config::load()?;
    config.ensure_storage_dir()?;
    
    // Sync database with filesystem on startup
    PackageDatabase::sync(&config.package_storage_path)
        .context("Failed to sync database")?;

    match cli.command {
        Commands::Install { github, branch, path } => {
            let installer = PackageInstaller::new(config.clone());
            
            if let Some(repo) = github {
                let package_name = installer.install_from_github(&repo, branch.as_deref())?;
                println!("Successfully installed package: {}", package_name);
            } else if let Some(source_path) = path {
                let package_name = installer.install_from_path(&source_path)?;
                println!("Successfully installed package: {}", package_name);
            } else {
                eprintln!("Error: Either --github or --path must be specified");
                std::process::exit(1);
            }
        }
        Commands::List => {
            let query = PackageQuery::new(config);
            let packages = query.list_all()?;
            if packages.is_empty() {
                println!("No packages installed.");
            } else {
                println!("Installed packages:");
                for pkg in packages {
                    println!("  - {}", pkg);
                }
            }
        }
        Commands::Info { name } => {
            let query = PackageQuery::new(config);
            query.show_info(&name)?;
        }
        Commands::SearchCap { name } => {
            let query = PackageQuery::new(config);
            let packages = query.find_by_capability(&name)?;
            if packages.is_empty() {
                println!("No packages found with capability: {}", name);
            } else {
                println!("Packages with capability '{}':", name);
                for pkg in packages {
                    println!("  - {}", pkg);
                }
            }
        }
        Commands::SearchSkill { name } => {
            let query = PackageQuery::new(config);
            let packages = query.find_by_skill(&name)?;
            if packages.is_empty() {
                println!("No packages found with skill: {}", name);
            } else {
                println!("Packages with skill '{}':", name);
                for pkg in packages {
                    println!("  - {}", pkg);
                }
            }
        }
        Commands::Register { recipe } => {
            let registrar = PackageRegistrar::new(config);
            registrar.register_from_recipe(&recipe).await?;
        }
        Commands::Config { set_storage_path, show } => {
            if let Some(new_path) = set_storage_path {
                let mut config = Config::load()?;
                config.package_storage_path = new_path;
                config.save()?;
                config.ensure_storage_dir()?;
                println!("Package storage path updated to: {}", config.package_storage_path.display());
            } else if show {
                let config = Config::load()?;
                println!("Package storage path: {}", config.package_storage_path.display());
            } else {
                eprintln!("Error: Either --set-storage-path or --show must be specified");
                std::process::exit(1);
            }
        }
    }

    Ok(())
}

