// SPDX-License-Identifier: MulanPSL-2.0
// Soma proto codegen: ROS IDL + contract TOML -> .proto -> tonic stubs.

use std::collections::BTreeSet;
use std::path::PathBuf;

use robonix_codegen::codegen::{contract_gen, msg_parser, proto_gen};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let repo_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .ok_or("could not locate repo root from CARGO_MANIFEST_DIR")?
        .to_path_buf();

    let idl_root = repo_root.join("capabilities/lib");
    let contracts_root = repo_root.join("capabilities/system/soma");
    let lifecycle_driver = repo_root.join("capabilities/lifecycle/driver.v1.toml");
    let proto_out = PathBuf::from(std::env::var("OUT_DIR")?);
    let selected_contracts = proto_out.join("selected_contracts");

    println!("cargo:rerun-if-changed={}", idl_root.display());
    println!("cargo:rerun-if-changed={}", contracts_root.display());
    println!("cargo:rerun-if-changed={}", lifecycle_driver.display());
    println!("cargo:rerun-if-changed=build.rs");
    let _ = std::fs::remove_dir_all(&selected_contracts);
    std::fs::create_dir_all(&selected_contracts)?;
    for name in [
        "get_yaml.v1.toml",
        "get_urdf.v1.toml",
        "footprint.v1.toml",
        "get_health.v1.toml",
        "health.v1.toml",
    ] {
        std::fs::copy(contracts_root.join(name), selected_contracts.join(name))?;
    }
    std::fs::copy(
        lifecycle_driver,
        selected_contracts.join("lifecycle_driver.v1.toml"),
    )?;

    let mut resolver = msg_parser::MsgResolver::new(std::slice::from_ref(&idl_root))?;
    let mut idl_skips = 0usize;
    resolver.resolve_all_in_index(false, &mut idl_skips)?;
    resolver.resolve_all_srv(false, &mut idl_skips)?;

    let contract_srvs: BTreeSet<(String, String)> =
        contract_gen::collect_referenced_srvs(&selected_contracts)?;
    proto_gen::generate(&resolver, &proto_out, Some(&contract_srvs), false)?;
    contract_gen::generate(
        &mut resolver,
        std::slice::from_ref(&selected_contracts),
        &proto_out,
        false,
    )?;

    let protoc = protoc_bin_vendored::protoc_bin_path()?;
    // SAFETY: build.rs is single-threaded.
    unsafe {
        std::env::set_var("PROTOC", protoc);
    }

    let proto_files: Vec<PathBuf> = std::fs::read_dir(&proto_out)?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().map(|x| x == "proto").unwrap_or(false))
        .collect();

    tonic_prost_build::configure()
        .build_server(true)
        .build_client(true)
        .compile_protos(&proto_files, std::slice::from_ref(&proto_out))?;

    Ok(())
}
