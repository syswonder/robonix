// SPDX-License-Identifier: MulanPSL-2.0
// Pilot proto codegen: ROS IDL + contract TOML → .proto (via robonix-codegen
// as a library) → tonic Rust stubs. All artefacts in OUT_DIR; nothing is
// committed to the tree. Atlas wire types are NOT regenerated here — pilot
// reaches them via `robonix_atlas::pb` from the atlas crate.

use std::collections::BTreeSet;
use std::path::PathBuf;

use robonix_codegen::codegen::{contract_gen, msg_parser, proto_gen};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    // system/pilot → up 2 = repo root.
    let repo_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .ok_or("could not locate repo root from CARGO_MANIFEST_DIR")?
        .to_path_buf();

    let idl_root = repo_root.join("capabilities/lib");
    let contracts_root = repo_root.join("capabilities");
    let proto_out = PathBuf::from(std::env::var("OUT_DIR")?);

    clean_generated_proto_dir(&proto_out)?;

    println!("cargo:rerun-if-changed={}", idl_root.display());
    println!("cargo:rerun-if-changed={}", contracts_root.display());
    println!("cargo:rerun-if-changed=build.rs");

    // 1. ROS IDL (.msg/.srv) + contract TOML → .proto in OUT_DIR.
    let mut resolver = msg_parser::MsgResolver::new(std::slice::from_ref(&idl_root))?;
    let mut idl_skips = 0usize;
    resolver.resolve_all_in_index(false, &mut idl_skips)?;
    resolver.resolve_all_srv(false, &mut idl_skips)?;

    let contract_srvs: BTreeSet<(String, String)> =
        contract_gen::collect_referenced_srvs(&contracts_root)?;
    proto_gen::generate(&resolver, &proto_out, Some(&contract_srvs), false)?;
    contract_gen::generate(
        &mut resolver,
        std::slice::from_ref(&contracts_root),
        &proto_out,
        false,
    )?;

    // 2. Compile every emitted .proto with tonic-prost-build (server+client).
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

/// Removes stale generated proto/Rust files from OUT_DIR before regenerating.
///
/// Side effect: deletes only files with `.proto` or `.rs` extensions in the
/// current crate build output directory. This keeps renamed IDL packages from
/// being compiled alongside their previous generated names during incremental
/// builds and rust-analyzer checks.
fn clean_generated_proto_dir(
    proto_out: &std::path::Path,
) -> Result<(), Box<dyn std::error::Error>> {
    if !proto_out.exists() {
        return Ok(());
    }
    for entry in std::fs::read_dir(proto_out)? {
        let path = entry?.path();
        let Some(ext) = path.extension().and_then(|x| x.to_str()) else {
            continue;
        };
        if matches!(ext, "proto" | "rs") {
            std::fs::remove_file(path)?;
        }
    }
    Ok(())
}
