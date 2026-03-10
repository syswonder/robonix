// SPDX-License-Identifier: MulanPSL-2.0
// ridlc - RIDL compiler PoC

use anyhow::{bail, Context, Result};
use clap::Parser;
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

// NOTE: this bin crate depends on the ridlc library crate.
// We must import modules through the library root, not `crate::`.
use ridlc::codegen::python_gen;
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

    /// Target language: python (default)
    #[arg(long = "lang", default_value = "python")]
    lang: String,
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

    eprintln!("[ridlc] includes: {:?}", args.include);
    eprintln!("[ridlc] out: {}", args.out.display());
    eprintln!("[ridlc] inputs (expanded):");
    for p in &all_inputs {
        eprintln!("  - {}", p.display());
    }

    // Group parsed files by RIDL namespace so each namespace becomes its own Python package
    // (e.g. robonix/prm/base vs robonix/prm/localization).
    let mut files_by_ns: HashMap<String, ridlc::ast::File> = HashMap::new();
    for path in &all_inputs {
        eprintln!("[ridlc] parsing RIDL: {}", path.display());
        let content = std::fs::read_to_string(path).with_context(|| {
            format!("failed to read RIDL file '{}'", path.display())
        })?;
        let file = parse_file(&content)
            .with_context(|| format!("failed to parse RIDL file '{}'", path.display()))?;

        let ns_key = file
            .namespace
            .clone()
            .unwrap_or_else(|| "robonix/unknown".to_string());

        use std::collections::hash_map::Entry;
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
            // Emit gRPC runtime client into out_dir once so code works out of the box.
            python_gen::emit_runtime_grpc(&args.out)?;
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

                python_gen::generate(ast, &args.out)?;
                eprintln!(
                    "[ridlc] generated Python stubs for namespace '{}' under '{}'",
                    ns,
                    args.out.display()
                );
            }
            // One ROS2 package per out_dir (ament_python); ready for package.xml / colcon.
            python_gen::emit_ros_package_files(&args.out, "robonix_interfaces")?;
        }
        _ => anyhow::bail!("unsupported --lang: {} (use 'python')", args.lang),
    }

    let total_ifaces = total_streams + total_commands + total_queries + total_events;
    eprintln!("[ridlc] summary:");
    eprintln!("  namespaces: {}", ns_count);
    eprintln!("  interfaces: {} (stream={}, command={}, query={}, event={})",
        total_ifaces, total_streams, total_commands, total_queries, total_events);

    Ok(())
}
