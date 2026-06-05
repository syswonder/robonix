// SPDX-License-Identifier: MulanPSL-2.0
// robonix-codegen — ROS IDL + contract codegen

use anyhow::{Result, bail};
use clap::Parser;
use std::collections::BTreeSet;
use std::io::{self, IsTerminal};
use std::path::PathBuf;

use robonix_codegen::codegen::{contract_gen, mcp_python_gen, msg_parser, proto_gen, ros2_gen};

#[derive(Parser)]
#[command(name = "robonix-codegen")]
#[command(
    about = "ROS IDL + contract TOML codegen: .msg/.srv → .proto / MCP-Python; contracts → robonix_contracts.proto"
)]
struct Args {
    /// Type search path for ROS IDL (e.g. -I path/to/robonix-interfaces/lib). Optional if only `--contracts` with protobuf-only types (rare).
    #[arg(short = 'I', long = "include", number_of_values = 1)]
    include: Vec<PathBuf>,

    /// One or more directories of contract `.toml` files (e.g.
    /// <root>/capabilities and <pkg>/capabilities). Pass multiple times
    /// to merge per-package contracts with the global tree. Emits
    /// `robonix_contracts.proto` (proto only).
    #[arg(long = "contracts", number_of_values = 1)]
    contracts: Vec<PathBuf>,

    /// Output directory for generated code
    #[arg(short = 'o', long = "out")]
    out: PathBuf,

    /// Target language: "proto" (gRPC stubs + contracts), "mcp"
    /// (Python typed-input helpers for MCP servers), or "ros2"
    /// (a colcon overlay of the canonical ROS interface packages).
    #[arg(long = "lang", default_value = "proto")]
    lang: String,

    /// Print per-package lines and every IDL resolution warning (default: one summary line; set ROBONIX_CODEGEN_VERBOSE=1 for same without flag)
    #[arg(long, short = 'v')]
    verbose: bool,
}

fn ridlc_prefix() -> &'static str {
    if io::stderr().is_terminal() {
        "\x1b[1;38;5;45m[robonix-codegen]\x1b[0m"
    } else {
        "[robonix-codegen]"
    }
}

fn verbose_from_args_and_env(args: &Args) -> bool {
    if args.verbose {
        return true;
    }
    std::env::var_os("ROBONIX_CODEGEN_VERBOSE").is_some_and(|v| {
        let s = v.to_string_lossy();
        s == "1" || s.eq_ignore_ascii_case("true")
    })
}

fn main() -> Result<()> {
    env_logger::init();
    let args = Args::parse();
    let verbose = verbose_from_args_and_env(&args);

    if args.include.is_empty() && args.contracts.is_empty() {
        bail!("[robonix-codegen] pass at least one -I/--include and/or --contracts (see --help)");
    }

    match args.lang.as_str() {
        "proto" | "mcp" | "ros2" => {}
        other => {
            bail!(
                "[robonix-codegen] unsupported --lang '{other}'. Supported: 'proto', 'mcp', 'ros2'."
            )
        }
    }

    if verbose {
        eprintln!("{} includes: {:?}", ridlc_prefix(), args.include);
        eprintln!(
            "{} contracts: {:?}",
            ridlc_prefix(),
            args.contracts
                .iter()
                .map(|p| p.display().to_string())
                .collect::<Vec<_>>()
        );
        eprintln!("{} out: {}", ridlc_prefix(), args.out.display());
    }

    std::fs::create_dir_all(&args.out)?;

    let mut resolver = msg_parser::MsgResolver::new(&args.include)?;
    let mut idl_skips = 0usize;
    resolver.resolve_all_in_index(verbose, &mut idl_skips)?;
    resolver.resolve_all_srv(verbose, &mut idl_skips)?;

    if !verbose && idl_skips > 0 {
        eprintln!(
            "{} {} IDL msg/srv skipped (unresolved deps); use --verbose for details",
            ridlc_prefix(),
            idl_skips
        );
    }

    let contract_srvs: Option<BTreeSet<(String, String)>> = if args.contracts.is_empty() {
        None
    } else {
        let mut set = BTreeSet::new();
        for cdir in &args.contracts {
            for pair in contract_gen::collect_referenced_srvs(cdir)? {
                set.insert(pair);
            }
        }
        Some(set)
    };

    let specs = resolver.ordered_specs();
    let srvs = resolver.ordered_srvs();
    let mut pkgs = std::collections::BTreeSet::new();
    for s in &specs {
        pkgs.insert(&s.package);
    }
    for s in &srvs {
        pkgs.insert(&s.package);
    }

    match args.lang.as_str() {
        "mcp" => {
            mcp_python_gen::generate(&resolver, &args.out, verbose)?;
            eprintln!(
                "{} mcp: {} packages, {} msgs -> {}",
                ridlc_prefix(),
                pkgs.len(),
                specs.len(),
                args.out.display()
            );
        }
        "ros2" => {
            ros2_gen::generate(&resolver, &args.out, verbose)?;
        }
        _ => {
            proto_gen::generate(&resolver, &args.out, contract_srvs.as_ref(), verbose)?;
            eprintln!(
                "{} proto: {} packages, {} msgs, {} srv -> {}",
                ridlc_prefix(),
                pkgs.len(),
                specs.len(),
                srvs.len(),
                args.out.display()
            );
            if !args.contracts.is_empty() {
                contract_gen::generate(&mut resolver, &args.contracts, &args.out, verbose)?;
                if !verbose {
                    eprintln!(
                        "{} contracts: robonix_contracts.proto + contract_proto_modules.rs (under {})",
                        ridlc_prefix(),
                        args.out.display()
                    );
                }
            }
        }
    }

    Ok(())
}
