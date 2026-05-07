// SPDX-License-Identifier: MulanPSL-2.0
// Command Module
//
// Command definitions and execution for robonix-cli

use anyhow::Result;
use clap::Subcommand;
use std::path::PathBuf;

use robonix_cli::Config;

mod ask;
mod build;
mod chat;
mod codegen;
mod config;
mod deploy;
mod info;
mod init;
mod inspect;
mod install;
mod list;
mod package_new;
mod path;
mod run_package;
mod setup;
mod shutdown;
mod teardown;
mod validate;

const DEFAULT_ENDPOINT: &str = "localhost:50051";

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
    /// Boot the whole stack from a top-level `robonix_manifest.yaml` — system
    /// services (atlas/executor/pilot/liaison/memory/vlm) plus every package
    /// declared under `primitive`/`service`/`skill`. Blocks until Ctrl-C.
    /// `rbnx deploy` is kept as an alias for back-compat.
    #[command(alias = "deploy")]
    Boot {
        /// Path to the deployment manifest (default: `./robonix_manifest.yaml`).
        #[arg(short = 'f', long, default_value = "robonix_manifest.yaml")]
        file: PathBuf,
        /// Directory for per-component logs (default: `<manifest-dir>/rbnx-boot/logs`).
        #[arg(long)]
        log_dir: Option<PathBuf>,
        /// Skip starting the `system:` block (atlas/pilot/etc). Useful when
        /// those are already running externally.
        #[arg(long)]
        skip_system: bool,
    },
    /// Tear down a stack previously brought up by `rbnx boot`. Reads the
    /// per-manifest state file boot writes (`<manifest-dir>/rbnx-boot/state.json`)
    /// to know which process groups + docker containers to kill, so the
    /// host doesn't accumulate orphaned drivers when boot dies on an error
    /// path or its parent shell window is closed.
    Shutdown {
        /// Path to the deployment manifest (default: `./robonix_manifest.yaml`).
        #[arg(short = 'f', long, default_value = "robonix_manifest.yaml")]
        file: PathBuf,
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
    /// Call once from a cloned robonix repo so packages anywhere on disk can find capabilities/IDL.
    Setup {
        /// Path to the robonix repo root (default: $RBNX_INVOCATION_CWD or process cwd).
        /// If the given path is a sub-directory, walks up to find the root.
        path: Option<PathBuf>,
    },
    /// Print an absolute path rooted in the configured robonix source tree (for build scripts).
    /// Keys: root, rust, capabilities, interfaces-lib, runtime-proto, robonix-api
    Path {
        /// Path key to resolve (see above).
        key: String,
    },

    /// List all registered capabilities (one row per cap by default;
    /// pass -v to expand the per-cap interface list, lspci -tv style)
    #[command(alias = "nodes")]
    Caps {
        /// robonix-atlas endpoint
        #[arg(long, env = "ROBONIX_ATLAS", default_value = DEFAULT_ENDPOINT)]
        server: String,
        /// Output as JSON (forces full detail regardless of -v)
        #[arg(long)]
        json: bool,
        /// Expand each cap's interface list; without this, only the
        /// summary header line per cap is printed.
        #[arg(short = 'v', long)]
        verbose: bool,
    },
    /// Show CAPABILITY.md for registered caps (all, or one with --cap)
    Describe {
        /// robonix-atlas endpoint
        #[arg(long, env = "ROBONIX_ATLAS", default_value = DEFAULT_ENDPOINT)]
        server: String,
        /// Show full CAPABILITY.md content for a specific cap_id
        #[arg(long, alias = "node")]
        cap: Option<String>,
        /// Output as JSON
        #[arg(long)]
        json: bool,
    },
    /// Print every MCP-callable tool visible to the agent (executor builtins + cap interfaces)
    Tools {
        /// robonix-atlas endpoint
        #[arg(long, env = "ROBONIX_ATLAS", default_value = DEFAULT_ENDPOINT)]
        server: String,
        /// Output as JSON
        #[arg(long)]
        json: bool,
    },
    /// Show active channels (consumer→provider connections opened via ConnectCapability)
    Channels {
        /// robonix-atlas endpoint
        #[arg(long, env = "ROBONIX_ATLAS", default_value = DEFAULT_ENDPOINT)]
        server: String,
    },
    /// Dump full runtime state as JSON (capabilities, interfaces, channels)
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

    /// Initialize a new robonix project (creates robonix_manifest.yaml + directory skeleton)
    Init {
        /// Project name (also used as directory name)
        name: String,
        /// Parent directory (default: current directory)
        #[arg(long)]
        path: Option<PathBuf>,
    },

    /// Create a new package under the appropriate role directory
    PackageNew {
        /// Package name
        name: String,
        /// Package type: primitive, service, or skill
        #[arg(short = 't', long = "type", default_value = "service")]
        pkg_type: String,
        /// Target package directory to create (when given, --type is ignored)
        #[arg(long)]
        path: Option<PathBuf>,
    },

    /// One-shot non-interactive prompt — same gRPC path as `rbnx chat`
    /// (atlas connect → SubmitTask → stream PilotEvent), but prints
    /// events to stdout and exits when the stream closes. Useful for
    /// scripted tests / CI / agent-driven runs where stdout is the
    /// artifact.
    Ask {
        /// The user message to send to the pilot.
        prompt: String,
        /// robonix-atlas endpoint
        #[arg(long, env = "ROBONIX_ATLAS", default_value = DEFAULT_ENDPOINT)]
        server: String,
        /// Emit one JSON object per pilot event on stdout (line-delimited).
        /// Default is human-readable text with tool-call summaries.
        #[arg(long)]
        json: bool,
    },
}

pub async fn execute(command: Commands, config: Config) -> Result<()> {
    match command {
        Commands::Build {
            path,
            global,
            clean,
        } => run_package::execute_build(config, path, global, clean).await,
        Commands::Start { package, endpoint } => {
            run_package::execute_start(&config, package.as_deref(), endpoint.as_deref()).await
        }
        Commands::Boot {
            file,
            log_dir,
            skip_system,
        } => deploy::execute(config, file, log_dir, skip_system).await,
        Commands::Shutdown { file } => shutdown::execute(file).await,
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
        Commands::Caps {
            server,
            json,
            verbose,
        } => inspect::caps(&server, json, verbose).await,
        Commands::Describe { server, cap, json } => {
            inspect::describe(&server, cap.as_deref(), json).await
        }
        Commands::Tools { server, json } => inspect::tools(&server, json).await,
        Commands::Channels { server } => inspect::channels(&server).await,
        Commands::Inspect { server } => inspect::inspect(&server).await,
        Commands::Chat { server } => chat::execute(&server).await,
        Commands::Init { name, path } => init::execute(&name, path.as_deref()).await,
        Commands::PackageNew {
            name,
            pkg_type,
            path,
        } => package_new::execute(&name, &pkg_type, path.as_deref()).await,
        Commands::Ask {
            prompt,
            server,
            json,
        } => ask::execute(&server, &prompt, json).await,
    }
}
