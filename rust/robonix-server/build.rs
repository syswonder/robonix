use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

fn collect_ridl_files(
    root: &Path,
    acc: &mut Vec<PathBuf>,
) -> Result<(), Box<dyn std::error::Error>> {
    if !root.exists() {
        return Ok(());
    }
    for entry in fs::read_dir(root)? {
        let entry = entry?;
        let path = entry.path();
        if path.is_dir() {
            collect_ridl_files(&path, acc)?;
        } else if path.extension().and_then(|s| s.to_str()) == Some("ridl") {
            acc.push(path);
        }
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir.parent().unwrap_or(&manifest_dir);
    let ridlc_proto = workspace_root.join("ridlc/proto");
    let robonix_interfaces = workspace_root.join("robonix-interfaces");

    println!("cargo:rerun-if-changed={}", ridlc_proto.join("robonix_runtime.proto").display());
    println!("cargo:rerun-if-changed={}", robonix_interfaces.join("ridl").display());
    println!("cargo:rerun-if-changed={}", robonix_interfaces.join("lib").display());

    let protoc = protoc_bin_vendored::protoc_bin_path()?;
    unsafe {
        std::env::set_var("PROTOC", protoc);
    }

    tonic_prost_build::configure()
        .build_server(true)
        .build_client(true)
        .compile_protos(
            &[ridlc_proto.join("robonix_runtime.proto")],
            &[ridlc_proto.clone()],
        )?;

    let mut ridl_files = Vec::new();
    collect_ridl_files(&robonix_interfaces.join("ridl"), &mut ridl_files)?;
    ridl_files.sort();

    let mut files_by_ns: BTreeMap<String, ridlc::ast::File> = BTreeMap::new();
    for path in ridl_files {
        let src = fs::read_to_string(&path)?;
        let file = ridlc::parser::parse_file(&src)?;
        let ns_key = file
            .namespace
            .clone()
            .unwrap_or_else(|| "robonix/unknown".to_string());
        use std::collections::btree_map::Entry;
        match files_by_ns.entry(ns_key) {
            Entry::Occupied(mut entry) => entry.get_mut().merge(file),
            Entry::Vacant(entry) => {
                entry.insert(file);
            }
        }
    }

    let generated_file = manifest_dir
        .join("src")
        .join("generated")
        .join("ridl_generated.rs");
    ridlc::codegen::rust_gen::generate_bindings(
        &files_by_ns,
        &[robonix_interfaces.join("lib")],
        &generated_file,
    )?;

    let rosidl_workspace = PathBuf::from("target").join("rclrs_interfaces_ws");
    let python_pkg_dir = rosidl_workspace.join("python_pkg");
    let workspace_src = rosidl_workspace.join("src");
    if python_pkg_dir.exists() {
        fs::remove_dir_all(&python_pkg_dir)?;
    }
    fs::create_dir_all(&python_pkg_dir)?;
    fs::create_dir_all(&workspace_src)?;
    ridlc::codegen::python_gen::emit_runtime_grpc(&python_pkg_dir)?;
    for file in files_by_ns.values() {
        ridlc::codegen::python_gen::generate(file, &python_pkg_dir)?;
    }
    ridlc::codegen::python_gen::assemble_workspace_ros_packages(
        &workspace_src,
        &python_pkg_dir,
        &[robonix_interfaces.join("lib")],
    )?;

    Ok(())
}
