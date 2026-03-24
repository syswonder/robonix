// SPDX-License-Identifier: MulanPSL-2.0
// ridlc - RIDL compiler PoC

use anyhow::{Context, Result, bail};
use clap::Parser;
use std::collections::BTreeMap;
use std::fs;
use std::io::{self, IsTerminal};
use std::path::{Path, PathBuf};

// NOTE: this bin crate depends on the ridlc library crate.
// We must import modules through the library root, not `crate::`.
use ridlc::codegen::{msg_parser, proto_gen, python_gen, rust_gen};
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

    /// Output directory for generated code (required unless --package-output is set)
    #[arg(short = 'o', long = "out")]
    out: Option<PathBuf>,

    /// Target language: python, rust, or proto
    #[arg(long = "lang", default_value = "python")]
    lang: String,

    /// Output layout: `workspace` emits a directly buildable ROS workspace,
    /// `package` keeps the legacy single-package + rosidl tree layout.
    #[arg(long = "layout", default_value = "workspace")]
    layout: String,

    /// Output path for package-local interfaces (when building a package with ridl/).
    /// When set, outputs to this path instead of workspace src/generated.
    #[arg(long = "package-output", value_name = "PATH")]
    package_output: Option<PathBuf>,

    /// Package name for generated interfaces (e.g. skill_demo_interfaces).
    /// Required when --package-output is set.
    #[arg(long = "package-name", value_name = "NAME")]
    package_name: Option<String>,
}

fn collect_ridl_from(path: &Path, acc: &mut Vec<PathBuf>) -> Result<()> {
    let meta =
        fs::metadata(path).with_context(|| format!("failed to stat path '{}'", path.display()))?;
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
        if path.extension().map(|e| e == "ridl").unwrap_or(false) {
            acc.push(path.to_path_buf());
        } else {
            bail!(
                "[ridlc] input path '{}' is not a .ridl file (expected .ridl extension)",
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
        anyhow::bail!(
            "[ridlc] at least one -I/--include path is required (e.g. -I path/to/robonix-interfaces/lib)"
        );
    }

    let package_mode = args.package_output.is_some();
    if package_mode {
        if args.package_name.is_none() {
            bail!("[ridlc] --package-name is required when --package-output is set");
        }
    } else if args.out.is_none() {
        bail!("[ridlc] -o/--out is required when not using --package-output");
    }
    let out_path = if package_mode {
        args.package_output.as_ref().unwrap().clone()
    } else {
        args.out.as_ref().unwrap().clone()
    };

    // Collect all RIDL inputs: positional + -i/--input (with directory expansion).
    let mut all_inputs: Vec<PathBuf> = Vec::new();
    for p in &args.inputs {
        all_inputs.push(p.clone());
    }
    for p in &args.input_paths {
        collect_ridl_from(p, &mut all_inputs)?;
    }
    if all_inputs.is_empty() && args.lang != "proto" {
        bail!(
            "[ridlc] no RIDL inputs provided.\n\
             Use positional paths or -i/--input <ridl-or-dir> (can repeat).\n\
             (For --lang proto, .ridl inputs are not required; .msg/.srv files are read from -I paths.)"
        );
    }

    eprintln!("{} includes: {:?}", ridlc_prefix(), args.include);
    eprintln!("{} out: {}", ridlc_prefix(), out_path.display());
    eprintln!("{} inputs (expanded):", ridlc_prefix());
    for p in &all_inputs {
        eprintln!("  - {}", p.display());
    }

    // Group parsed files by RIDL namespace so each namespace becomes its own Python package
    // (e.g. robonix/prm/base vs robonix/prm/localization).
    let mut files_by_ns: BTreeMap<String, ridlc::ast::File> = BTreeMap::new();
    for path in &all_inputs {
        eprintln!("{} parsing RIDL: {}", ridlc_prefix(), path.display());
        let content = std::fs::read_to_string(path)
            .with_context(|| format!("[ridlc] failed to read RIDL file '{}'", path.display()))?;
        let file = parse_file(&content)
            .with_context(|| format!("[ridlc] failed to parse RIDL file '{}'", path.display()))?;

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

    // Resolve short type names (e.g. "String") to full paths (e.g. "std_msgs/msg/String") using imports
    for ast in files_by_ns.values_mut() {
        ast.resolve_imports()
            .with_context(|| "[ridlc] failed to resolve type references from imports")?;
    }

    // Package-local mode: namespace must start with manifest package.name (e.g. skill_demo or skill_demo/xxx)
    if package_mode {
        let pkg_name = args.package_name.as_ref().unwrap();
        let ns_prefix = pkg_name.strip_suffix("_interfaces").unwrap_or(pkg_name);
        for ns in files_by_ns.keys() {
            let valid = ns == ns_prefix || ns.starts_with(&format!("{}/", ns_prefix));
            if !valid {
                bail!(
                    "[ridlc] package-local RIDL namespace '{}' must be '{}' or '{}/*' (from --package-name {})",
                    ns,
                    ns_prefix,
                    ns_prefix,
                    pkg_name
                );
            }
        }
    }

    std::fs::create_dir_all(&out_path)?;

    // Simple statistics for generated artifacts.
    let mut total_streams = 0usize;
    let mut total_commands = 0usize;
    let mut total_queries = 0usize;
    let mut ns_count = 0usize;

    match args.lang.as_str() {
        "python" => {
            let python_out = if package_mode {
                let pkg_name = args.package_name.as_ref().unwrap();
                let pkg_out = out_path.join(pkg_name);
                std::fs::create_dir_all(&pkg_out)?;
                pkg_out
            } else {
                match args.layout.as_str() {
                    "workspace" => {
                        let ws_src = out_path.join("src");
                        std::fs::create_dir_all(&ws_src)?;
                        for subdir in ["generated", "vendor"] {
                            let package_dir = ws_src.join(subdir);
                            if package_dir.exists() {
                                std::fs::remove_dir_all(&package_dir).with_context(|| {
                                    format!(
                                        "failed to remove previous package dir '{}'",
                                        package_dir.display()
                                    )
                                })?;
                            }
                        }
                        ws_src.join("generated").join("robonix_interfaces")
                    }
                    "package" => out_path.clone(),
                    _ => anyhow::bail!(
                        "unsupported --layout: {} (use 'workspace' or 'package')",
                        args.layout
                    ),
                }
            };

            let gen_options = if package_mode {
                Some(ridlc::codegen::python_gen::PackageGenOptions {
                    python_package_name: args.package_name.as_ref().unwrap().clone(),
                    rosidl_base: out_path.clone(),
                })
            } else {
                None
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
                    }
                }
                python_gen::generate(ast, &python_out, gen_options.as_ref())?;
                eprintln!(
                    "{} generated Python stubs for namespace '{}' under '{}'",
                    ridlc_prefix(),
                    ns,
                    python_out.display()
                );
            }
            let pkg_name_for_ros = if package_mode {
                args.package_name.as_ref().unwrap().as_str()
            } else {
                "robonix_interfaces"
            };
            python_gen::emit_ros_package_files(&python_out, pkg_name_for_ros)?;
            if package_mode {
                python_gen::emit_package_local_rosidl(
                    &out_path,
                    args.package_name.as_ref().unwrap(),
                    &files_by_ns,
                )?;
            } else if args.layout == "workspace" {
                let workspace_src = out_path.join("src");
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
                "workspace" | "package" => out_path.join("ridl_generated.rs"),
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
        "proto" => {
            let mut resolver = msg_parser::MsgResolver::new(&args.include)?;
            resolver.resolve_all_in_index()?;
            resolver.resolve_all_srv()?;
            let specs = resolver.ordered_specs();
            let srvs = resolver.ordered_srvs();
            let mut pkgs = std::collections::BTreeSet::new();
            for s in &specs {
                pkgs.insert(&s.package);
            }
            for s in &srvs {
                pkgs.insert(&s.package);
            }
            ns_count = pkgs.len();
            let type_count = specs.len();
            let srv_count = srvs.len();
            proto_gen::generate(&resolver, &out_path)?;
            eprintln!(
                "{} generated proto: {} packages, {} msgs, {} srvs -> {}",
                ridlc_prefix(),
                ns_count,
                type_count,
                srv_count,
                out_path.display()
            );
        }
        _ => anyhow::bail!(
            "unsupported --lang: {} (use 'python', 'rust', or 'proto')",
            args.lang
        ),
    }

    let total_ifaces = total_streams + total_commands + total_queries;
    eprintln!("{} summary:", ridlc_prefix());
    eprintln!("  namespaces: {}", ns_count);
    eprintln!(
        "  interfaces: {} (stream={}, command={}, query={})",
        total_ifaces, total_streams, total_commands, total_queries
    );

    Ok(())
}
