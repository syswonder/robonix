// SPDX-License-Identifier: MulanPSL-2.0
// robonix-codegen — ROS IDL + contract codegen

use anyhow::{Result, bail};
use clap::Parser;
use std::collections::BTreeSet;
use std::io::{self, IsTerminal};
use std::path::PathBuf;

use robonix_codegen::codegen::{contract_gen, mcp_python_gen, msg_parser, proto_gen, python_gen};

#[derive(Parser)]
#[command(name = "robonix-codegen")]
#[command(
    about = "ROS IDL + contract TOML codegen: .msg/.srv → .proto / Python; contracts → robonix_contracts.proto"
)]
struct Args {
    /// Type search path for ROS IDL (e.g. -I path/to/robonix-interfaces/lib). Optional if only `--contracts` with protobuf-only types (rare).
    #[arg(short = 'I', long = "include", number_of_values = 1)]
    include: Vec<PathBuf>,

    /// Directory of contract `.toml` files (e.g. rust/contracts). Emits `robonix_contracts.proto` (proto only).
    #[arg(long = "contracts")]
    contracts: Option<PathBuf>,

    /// Output directory for generated code
    #[arg(short = 'o', long = "out")]
    out: PathBuf,

    /// Target language: "proto" or "python"
    #[arg(long = "lang", default_value = "proto")]
    lang: String,
}

fn ridlc_prefix() -> &'static str {
    if io::stderr().is_terminal() {
        "\x1b[1;38;5;45m[robonix-codegen]\x1b[0m"
    } else {
        "[robonix-codegen]"
    }
}

fn main() -> Result<()> {
    env_logger::init();
    let args = Args::parse();

    if args.include.is_empty() && args.contracts.is_none() {
        bail!("[robonix-codegen] pass at least one -I/--include and/or --contracts (see --help)");
    }

    match args.lang.as_str() {
        "proto" | "python" | "mcp" => {}
        other => bail!(
            "[robonix-codegen] unsupported --lang '{other}'. Supported: 'proto', 'python', 'mcp'."
        ),
    }

    eprintln!("{} includes: {:?}", ridlc_prefix(), args.include);
    eprintln!(
        "{} contracts: {:?}",
        ridlc_prefix(),
        args.contracts.as_ref().map(|p| p.display().to_string())
    );
    eprintln!("{} out: {}", ridlc_prefix(), args.out.display());

    std::fs::create_dir_all(&args.out)?;

    let mut resolver = msg_parser::MsgResolver::new(&args.include)?;
    resolver.resolve_all_in_index()?;
    resolver.resolve_all_srv()?;

    let contract_srvs: Option<BTreeSet<(String, String)>> = match &args.contracts {
        Some(cdir) => Some(contract_gen::collect_referenced_srvs(cdir)?),
        None => None,
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
        "python" => {
            python_gen::generate(&resolver, &args.out)?;
            eprintln!(
                "{} generated python ctypes (c-mode): {} packages, {} msgs -> {}",
                ridlc_prefix(),
                pkgs.len(),
                specs.len(),
                args.out.display()
            );
        }
        "mcp" => {
            mcp_python_gen::generate(&resolver, &args.out)?;
            eprintln!(
                "{} generated python dataclasses (mcp-mode): {} packages, {} msgs -> {}",
                ridlc_prefix(),
                pkgs.len(),
                specs.len(),
                args.out.display()
            );
        }
        _ => {
            proto_gen::generate(&resolver, &args.out, contract_srvs.as_ref())?;
            eprintln!(
                "{} generated proto: {} packages, {} msgs, {} srv indexed -> {}",
                ridlc_prefix(),
                pkgs.len(),
                specs.len(),
                srvs.len(),
                args.out.display()
            );
            if let Some(ref cdir) = args.contracts {
                contract_gen::generate(&mut resolver, cdir, &args.out)?;
            }
        }
    }

    Ok(())
}
