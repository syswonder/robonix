// SPDX-License-Identifier: MulanPSL-2.0
// ridlc - RIDL compiler PoC

use anyhow::{bail, Context, Result};
use clap::Parser;
use std::collections::BTreeMap;
use std::fs;
use std::io::{self, IsTerminal};
use std::path::{Path, PathBuf};

// NOTE: this bin crate depends on the ridlc library crate.
// We must import modules through the library root, not `crate::`.
use ridlc::codegen::{python_gen, rust_gen};
use ridlc::parser::parse_file;

#[derive(Parser)]
#[command(name = "ridlc")]
#[command(about = "RIDL compiler: parse .ridl and generate Python ROS2 code")]
struct Args {
    /// One or more .ridl files as positional arguments
    /// (can be combined with -i/--input).
    #[arg(required = false)]
    inputs: Vec<PathBuf>,

    /// RIDL file or directory (can be given multiple times).
    /// - If PATH is a file: added directly as a RIDL input
    /// - If PATH is a directory: recursively collect all *.ridl under it
    #[arg(short = 'i', long = "input", value_name = "PATH")]
    input_paths: Vec<PathBuf>,

    /// Type search path for resolving imports (e.g. -I path)
    #[arg(short = 'I', long = "include", number_of_values = 1)]
    include: Vec<PathBuf>,

    /// Output directory for generated code
    #[arg(short = 'o', long = "out", required = true)]
    out: PathBuf,

    /// Target language: python or rust
    #[arg(long = "lang", default_value = "python")]
    lang: String,

    /// Output layout: `workspace` emits a directly buildable ROS workspace,
    /// `package` keeps the legacy single-package + rosidl tree layout.
    #[arg(long = "layout", default_value = "workspace")]
    layout: String,
}

fn collect_ridl_from(path: &Path, acc: &mut Vec<PathBuf>) -> Result<()> {
    let meta = fs::metadata(path)
        .with_context(|| format!("failed to stat path '{}'", path.display()))?;
    if meta.is_dir() {
        for entry in fs::read_dir(path)
            .with_context(|| format!("failed to read dir '{}'", path.display()))?
        {
            let entry = entry?;
            let p = entry.path();
            if p.is_dir() {
                collect_ridl_from(&p, acc)?;
            } else if let Some(ext) = p.extension() {
                if ext == "ridl" {
                    acc.push(p);
                }
            }
        }
    } else {
        // Single file: require .ridl extension
        if path
            .extension()
            .map(|e| e == "ridl")
            .unwrap_or(false)
        {
            acc.push(path.to_path_buf());
        } else {
            bail!(
                "input path '{}' is not a .ridl file",
                path.display()
            );
        }
    }
    Ok(())
}

fn ridlc_prefix() -> &'static str {
    if io::stderr().is_terminal() {
        "\x1b[1;38;5;45m[ridlc]\x1b[0m"
    } else {
        "[ridlc]"
    }
}

fn main() -> Result<()> {
    env_logger::init();
    let args = Args::parse();

    if args.include.is_empty() {
        anyhow::bail!("at least one -I/--include path is required");
    }

    // Collect all RIDL inputs: positional + -i/--input (with directory expansion).
    let mut all_inputs: Vec<PathBuf> = Vec::new();
    for p in &args.inputs {
        all_inputs.push(p.clone());
    }
    for p in &args.input_paths {
        collect_ridl_from(p, &mut all_inputs)?;
    }
    if all_inputs.is_empty() {
        bail!(
            "no RIDL inputs provided.\n\
             Use positional paths or -i/--input <ridl-or-dir> (can repeat)."
        );
    }

    eprintln!("{} includes: {:?}", ridlc_prefix(), args.include);
    eprintln!("{} out: {}", ridlc_prefix(), args.out.display());
    eprintln!("{} inputs (expanded):", ridlc_prefix());
    for p in &all_inputs {
        eprintln!("  - {}", p.display());
    }

    // Group parsed files by RIDL namespace so each namespace becomes its own Python package
    // (e.g. robonix/prm/base vs robonix/prm/localization).
    let mut files_by_ns: BTreeMap<String, ridlc::ast::File> = BTreeMap::new();
    for path in &all_inputs {
        eprintln!("{} parsing RIDL: {}", ridlc_prefix(), path.display());
        let content = std::fs::read_to_string(path).with_context(|| {
            format!("failed to read RIDL file '{}'", path.display())
        })?;
        let file = parse_file(&content)
            .with_context(|| format!("failed to parse RIDL file '{}'", path.display()))?;

        let ns_key = file
            .namespace
            .clone()
            .unwrap_or_else(|| "robonix/unknown".to_string());

        use std::collections::btree_map::Entry;
        match files_by_ns.entry(ns_key) {
            Entry::Occupied(mut e) => {
                e.get_mut().merge(file);
            }
            Entry::Vacant(e) => {
                e.insert(file);
            }
        }
    }

    std::fs::create_dir_all(&args.out)?;

    // Simple statistics for generated artifacts.
    let mut total_streams = 0usize;
    let mut total_commands = 0usize;
    let mut total_queries = 0usize;
    let mut total_events = 0usize;
    let mut ns_count = 0usize;

    match args.lang.as_str() {
        "python" => {
            let python_out = match args.layout.as_str() {
                "workspace" => {
                    let ws_src = args.out.join("src");
                    std::fs::create_dir_all(&ws_src)?;
                    for subdir in ["generated", "vendor"] {
                        let package_dir = ws_src.join(subdir);
                        if package_dir.exists() {
                            std::fs::remove_dir_all(&package_dir).with_context(|| {
                                format!("failed to remove previous package dir '{}'", package_dir.display())
                            })?;
                        }
                    }
                    ws_src.join("generated").join("robonix_interfaces")
                }
                "package" => args.out.clone(),
                _ => anyhow::bail!(
                    "unsupported --layout: {} (use 'workspace' or 'package')",
                    args.layout
                ),
            };

            // Emit gRPC runtime client into the generated runtime package once
            // so code works out of the box.
            python_gen::emit_runtime_grpc(&python_out)?;
            for (ns, ast) in &files_by_ns {
                ns_count += 1;
                for iface in &ast.interfaces {
                    match iface {
                        ridlc::ast::Interface::Stream(_) => total_streams += 1,
                        ridlc::ast::Interface::Command(_) => total_commands += 1,
                        ridlc::ast::Interface::Query(_) => total_queries += 1,
                        ridlc::ast::Interface::Event(_) => total_events += 1,
                    }
                }
                python_gen::generate(ast, &python_out)?;
                eprintln!(
                    "{} generated Python stubs for namespace '{}' under '{}'",
                    ridlc_prefix(),
                    ns,
                    python_out.display()
                );
            }
            python_gen::emit_ros_package_files(&python_out, "robonix_interfaces")?;
            if args.layout == "workspace" {
                let workspace_src = args.out.join("src");
                python_gen::assemble_workspace_ros_packages(
                    &workspace_src,
                    &python_out,
                    &args.include,
                )?;
                python_gen::emit_app_skeleton(&workspace_src, &files_by_ns)?;
            }
        }
        "rust" => {
            let rust_out = match args.layout.as_str() {
                "workspace" | "package" => args.out.join("ridl_generated.rs"),
                _ => anyhow::bail!(
                    "unsupported --layout: {} (use 'workspace' or 'package')",
                    args.layout
                ),
            };
            for (ns, ast) in &files_by_ns {
                ns_count += 1;
                for iface in &ast.interfaces {
                    match iface {
                        ridlc::ast::Interface::Stream(_) => total_streams += 1,
                        ridlc::ast::Interface::Command(_) => total_commands += 1,
                        ridlc::ast::Interface::Query(_) => total_queries += 1,
                        ridlc::ast::Interface::Event(_) => total_events += 1,
                    }
                }
                eprintln!(
                    "{} queued Rust bindings for namespace '{}'",
                    ridlc_prefix(),
                    ns
                );
            }
            rust_gen::generate_bindings(&files_by_ns, &args.include, &rust_out)?;
            eprintln!(
                "{} generated Rust bindings under '{}'",
                ridlc_prefix(),
                rust_out.display()
            );
        }
        _ => anyhow::bail!("unsupported --lang: {} (use 'python' or 'rust')", args.lang),
    }

    let total_ifaces = total_streams + total_commands + total_queries + total_events;
    eprintln!("{} summary:", ridlc_prefix());
    eprintln!("  namespaces: {}", ns_count);
    eprintln!("  interfaces: {} (stream={}, command={}, query={}, event={})",
        total_ifaces, total_streams, total_commands, total_queries, total_events);

    Ok(())
}
