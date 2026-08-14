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
mod check_remotes;
mod clean;
mod codegen;
mod config;
mod deploy;
mod docs;
mod info;
mod init;
mod inspect;
mod install;
mod list;
mod logs;
mod package_new;
mod path;
mod run_package;
mod setup;
mod shutdown;
mod teardown;
mod update;
mod validate;

const DEFAULT_ENDPOINT: &str = "localhost:50051";

#[derive(Subcommand)]
pub enum Commands {
    /// Build a package (local path or system-installed)
    Build {
        /// Deployment manifest to build (builds every declared package)
        #[arg(short = 'f', long, value_name = "FILE", conflicts_with_all = ["path", "global"])]
        file: Option<PathBuf>,
        /// Local package path (relative to $RBNX_INVOCATION_CWD, else process cwd)
        #[arg(short = 'p', long)]
        path: Option<PathBuf>,
        /// Build by system-installed package name
        #[arg(short = 'g', long)]
        global: Option<String>,
        /// Clean build (remove rbnx-build before building). Default: incremental.
        #[arg(long)]
        clean: bool,
        /// Skip the remote-provider freshness check before building cached packages.
        /// Useful when offline or when the deployment cache is intentionally pinned.
        #[arg(long)]
        no_update_check: bool,
    },
    /// Start one package (runs its `start` block; blocks until it exits)
    ///
    /// Defaults to the package containing the current directory when `-p`
    /// is omitted. The pre-dev-packaging `-n / --node` flag is gone — one
    /// package = one start body now.
    Start {
        /// Package path or installed name; relative paths use $RBNX_INVOCATION_CWD, else process cwd.
        /// If omitted, rbnx walks up from the current directory to find a package manifest.
        #[arg(short = 'p', long)]
        package: Option<String>,
        /// Registry endpoint (default: 127.0.0.1:50051)
        #[arg(long)]
        endpoint: Option<String>,
        /// Per-instance config file (JSON or YAML). Materialized into
        /// `RBNX_CONFIG_FILE` for the start body. Same shape as the per-
        /// package `config:` block under a deploy `robonix_manifest.yaml`
        /// — `rbnx boot` writes one of these per package and re-execs
        /// `rbnx start --config <file>` internally.
        #[arg(short = 'c', long)]
        config: Option<PathBuf>,
        /// Inline config overrides. Repeatable, dotted-path keys, e.g.
        /// `--set sensors.lidar2d=true --set algo=rtabmap`. Layered on
        /// top of `--config` (sets win). Values are JSON-parsed when
        /// possible (so `--set max_speed=0.5` is a number, `--set on=true`
        /// is a bool); fall back to a string when JSON parsing fails.
        #[arg(short = 's', long = "set", value_name = "KEY=VALUE")]
        set: Vec<String>,
        /// Package manifest filename to use instead of the default
        /// `package_manifest.yaml`. Lets a package ship per-deployment-target
        /// variants (e.g. `package_manifest.jetson-native.yaml`). `rbnx boot`
        /// passes this through from a deploy entry's `manifest:` field.
        #[arg(short = 'm', long)]
        manifest: Option<String>,
    },
    /// Boot the whole stack from a `robonix_manifest.yaml` (until Ctrl-C)
    ///
    /// Brings up declared built-in system components
    /// (atlas/executor/soma/pilot/vitals/liaison), other system packages, and
    /// every package declared under `primitive`/`service`/`skill`.
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
        /// Skip the remote-provider freshness check (the per-package
        /// `git fetch` pass that runs before boot). Use when offline or in a
        /// hurry.
        #[arg(long)]
        no_update_check: bool,
        /// Stream append-only, Scribe-backed component logs during boot.
        /// Disables animated cursor updates so output can be read or piped
        /// like a Linux/FreeBSD kernel boot log or Android logcat.
        #[arg(short, long)]
        verbose: bool,
    },
    /// Update remote (`url:`) providers to their latest upstream commit
    ///
    /// In a deploy dir (or with `-f <manifest>`) updates ALL cloned remote
    /// providers; with `-p <dir>` (or inside a package checkout) updates just
    /// that one. Shows an overview and asks for confirmation before pulling.
    Update {
        /// Update a single package checkout at this path.
        #[arg(short = 'p', long)]
        path: Option<PathBuf>,
        /// Deploy manifest whose remote providers to update
        /// (default: `./robonix_manifest.yaml`).
        #[arg(short = 'f', long)]
        file: Option<PathBuf>,
    },
    /// Tear down a stack previously brought up by `rbnx boot`
    ///
    /// Reads the per-manifest state file boot writes
    /// (`<manifest-dir>/rbnx-boot/state.json`) to kill the right process
    /// groups + docker containers, so the host doesn't accumulate orphaned
    /// drivers when boot dies on an error path or its shell window is closed.
    Shutdown {
        /// Path to the deployment manifest (default: `./robonix_manifest.yaml`).
        #[arg(short = 'f', long, default_value = "robonix_manifest.yaml")]
        file: PathBuf,
    },
    /// Drop build artifacts (`rbnx-build/`), per-package or per-deploy
    ///
    /// `rbnx clean -p <pkg>` removes `<pkg>/rbnx-build/`. `rbnx clean -f
    /// <manifest>` recurses over every package the manifest references
    /// (path: + url: + system/*), wipes each one's `rbnx-build/`, and clears
    /// the deploy's `rbnx-boot/{logs,state.json}`. `--cache` also wipes
    /// `rbnx-boot/cache/` (forces re-clone of url: packages). Defaults to the
    /// package containing cwd when neither `-p` nor `-f` is given.
    Clean {
        /// Package path (defaults to walking up from cwd).
        #[arg(short = 'p', long)]
        package: Option<PathBuf>,
        /// Deploy manifest path. When set, recurses over the manifest.
        #[arg(short = 'f', long)]
        file: Option<PathBuf>,
        /// With `-f`, also wipe `rbnx-boot/cache/` (force re-clone).
        #[arg(long)]
        cache: bool,
    },
    /// Install a package from GitHub or local path
    ///
    /// Legacy system-installed-package tooling; superseded by deploy
    /// manifests (`rbnx boot`/`build`). Hidden from the command list but
    /// still functional.
    #[command(hide = true)]
    Install {
        /// Install from GitHub (e.g. user/repo or https://github.com/user/repo)
        #[arg(long)]
        github: Option<String>,
        /// Install from local path
        #[arg(long)]
        path: Option<PathBuf>,
    },
    /// List system-installed packages (legacy; hidden)
    #[command(hide = true)]
    List,
    /// Show details of a system-installed package (legacy; hidden)
    #[command(hide = true)]
    Info {
        /// Package name
        name: String,
    },
    /// Validate a package manifest without building
    ///
    /// If no path is given, rbnx walks up from the current directory to find
    /// a package manifest.
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
    /// Run codegen for a package (proto + gRPC stubs + MCP types)
    ///
    /// Wraps robonix-codegen + Python grpcio-tools: stages system protos under
    /// `<pkg>/rbnx-build/proto-staging/`, then emits generated artifacts under
    /// `<pkg>/rbnx-build/codegen/`. If `-p` is omitted, rbnx walks up from the
    /// current directory to find a package manifest.
    Codegen {
        /// Package path (relative to $RBNX_INVOCATION_CWD, else process cwd)
        #[arg(short = 'p', long)]
        package: Option<PathBuf>,
        /// Also generate robonix_mcp_types/ (for MCP-based packages)
        #[arg(long)]
        mcp: bool,
        /// Also generate ros2_idl/ — the canonical ROS 2 message overlay
        /// (source). Build it with `colcon build` in a ROS 2 environment and
        /// source install/setup.bash so rclpy types are Robonix's.
        #[arg(long)]
        ros2: bool,
        /// Remove previous generated outputs before regenerating (never removes rbnx-build/ws/)
        #[arg(long)]
        clean: bool,
        /// Directory (relative to package root, or absolute) containing generated artifact directories.
        /// Defaults to `<package>/rbnx-build/codegen`.
        #[arg(long)]
        out_dir: Option<PathBuf>,
        /// Python interpreter used for grpcio-tools generation and import validation.
        /// Precedence: this flag, RBNX_CODEGEN_PYTHON, then `python3`.
        #[arg(long, value_name = "PATH")]
        python: Option<PathBuf>,
    },
    /// Regenerate the mdBook contract + ROS IDL reference
    ///
    /// Rebuilds `docs/src/reference/{contracts,idl}.md` from `capabilities/`.
    /// Auto-generated + version-stamped — run after changing any contract or
    /// IDL so the browsable reference stays in sync.
    Docs {
        /// Output directory (default: `<root>/docs/src/reference`).
        #[arg(long)]
        out_dir: Option<PathBuf>,
    },
    /// Register this directory as the robonix source tree
    ///
    /// Persists to ~/.robonix/config.yaml. Call once from a cloned robonix
    /// repo so packages anywhere on disk can find capabilities/IDL.
    Setup {
        /// Path to the robonix repo root (default: $RBNX_INVOCATION_CWD or process cwd).
        /// If the given path is a sub-directory, walks up to find the root.
        path: Option<PathBuf>,
    },
    /// Print an absolute path in the robonix source tree (for build scripts)
    ///
    /// Keys: root, rust, capabilities, interfaces-lib, runtime-proto, robonix-api.
    Path {
        /// Path key to resolve (see above).
        key: String,
    },

    /// List registered capabilities (one row per provider)
    ///
    /// Pass -v to expand the per-provider capability list, lspci -tv style.
    #[command(alias = "nodes")]
    Caps {
        /// robonix-atlas endpoint
        #[arg(long, env = "ROBONIX_ATLAS", default_value = DEFAULT_ENDPOINT)]
        server: String,
        /// Output as JSON (forces full detail regardless of -v)
        #[arg(long)]
        json: bool,
        /// Expand each provider's capability list; without this, only the
        /// summary header line per provider is printed.
        #[arg(short = 'v', long)]
        verbose: bool,
    },
    /// List atlas's loaded contract registry
    ///
    /// Every `<root>/capabilities/**/*.toml` atlas parsed at startup. -v for
    /// field-level schemas + source paths; -p/--prefix filters by namespace.
    Contracts {
        /// robonix-atlas endpoint
        #[arg(long, env = "ROBONIX_ATLAS", default_value = DEFAULT_ENDPOINT)]
        server: String,
        /// Filter by id prefix (e.g. `robonix/primitive/camera/`)
        #[arg(short = 'p', long)]
        prefix: Option<String>,
        /// Output as JSON (forces full detail)
        #[arg(long)]
        json: bool,
        /// Expand each contract's field schema + source toml path
        #[arg(short = 'v', long)]
        verbose: bool,
    },
    /// Show CAPABILITY.md for registered providers (all, or one with --provider)
    Describe {
        /// robonix-atlas endpoint
        #[arg(long, env = "ROBONIX_ATLAS", default_value = DEFAULT_ENDPOINT)]
        server: String,
        /// Show full CAPABILITY.md content for a specific provider_id
        #[arg(long, alias = "node")]
        provider: Option<String>,
        /// Output as JSON
        #[arg(long)]
        json: bool,
    },
    /// Print every MCP-callable tool visible to the agent (executor builtins + provider capabilities)
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
    /// Dump full runtime state as JSON (providers, capabilities, channels)
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

    /// Initialize a new robot deployment directory (creates robonix_manifest.yaml)
    Init {
        /// Robot deployment directory name
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

    /// One-shot non-interactive prompt to the agent (stdout, then exit)
    ///
    /// Same gRPC path as `rbnx chat` (atlas connect → SubmitTask → stream
    /// PilotEvent) but prints events to stdout and exits when the stream
    /// closes. Useful for scripted tests / CI / agent-driven runs.
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
    /// Read Scribe JSON-lines log files and render them with optional
    /// tag / level filtering.  Point at a log directory or read the
    /// default `<manifest-dir>/rbnx-boot/logs`.
    Logs {
        /// Log directory (default: `./rbnx-boot/logs` or `$SCRIBE_LOG_DIR`).
        #[arg(short = 'd', long)]
        log_dir: Option<PathBuf>,
        /// Filter to one or more tags (OR semantics).
        #[arg(short = 't', long)]
        tag: Vec<String>,
        /// Minimum level to show (debug < info < warn < error).
        #[arg(short = 'l', long)]
        level: Option<String>,
        /// Follow mode — keep reading new lines as they arrive (tail -f).
        #[arg(short = 'f', long)]
        follow: bool,
        /// Output raw JSON lines instead of logcat-style rendering.
        #[arg(long)]
        json: bool,
        /// List the distinct tags present in the logs (with record counts)
        /// instead of printing records — handy for discovering `-t` values.
        #[arg(long)]
        list_tags: bool,
    },
}

pub async fn execute(command: Commands, config: Config) -> Result<()> {
    match command {
        Commands::Build {
            file,
            path,
            global,
            clean,
            no_update_check,
        } => run_package::execute_build(config, file, path, global, clean, no_update_check).await,
        Commands::Start {
            package,
            endpoint,
            config: cfg_file,
            set,
            manifest,
        } => {
            run_package::execute_start(
                &config,
                package.as_deref(),
                endpoint.as_deref(),
                cfg_file.as_deref(),
                &set,
                manifest.as_deref(),
            )
            .await
        }
        Commands::Boot {
            file,
            log_dir,
            skip_system,
            no_update_check,
            verbose,
        } => deploy::execute(config, file, log_dir, skip_system, no_update_check, verbose).await,
        Commands::Update { path, file } => update::execute(config, path, file).await,
        Commands::Shutdown { file } => shutdown::execute(file).await,
        Commands::Clean {
            package,
            file,
            cache,
        } => clean::execute(config, package, file, cache).await,
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
            ros2,
            clean,
            out_dir,
            python,
        } => codegen::execute(config, package, mcp, ros2, clean, out_dir, python).await,
        Commands::Docs { out_dir } => docs::execute(config, out_dir).await,
        Commands::Setup { path } => setup::execute(config, path).await,
        Commands::Path { key } => path::execute(config, key).await,
        Commands::Caps {
            server,
            json,
            verbose,
        } => inspect::providers(&server, json, verbose).await,
        Commands::Contracts {
            server,
            prefix,
            json,
            verbose,
        } => inspect::contracts(&server, prefix.as_deref(), json, verbose).await,
        Commands::Describe {
            server,
            provider,
            json,
        } => inspect::describe(&server, provider.as_deref(), json).await,
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
        Commands::Logs {
            log_dir,
            tag,
            level,
            follow,
            json,
            list_tags,
        } => logs::execute(log_dir, tag, level, follow, json, list_tags).await,
    }
}
