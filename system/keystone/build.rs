// SPDX-License-Identifier: MulanPSL-2.0
// Keystone codegen: shared Robonix IDL + capability contracts -> proto -> tonic.

use std::collections::BTreeSet;
use std::path::PathBuf;

use robonix_codegen::codegen::{contract_gen, msg_parser, proto_gen};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let repo_root = manifest_dir
        .parent()
        .and_then(|path| path.parent())
        .ok_or("could not locate repo root from CARGO_MANIFEST_DIR")?
        .to_path_buf();
    let idl_root = repo_root.join("capabilities/lib");
    // Keystone implements the account contracts and consumes Voiceprint
    // contracts during enrollment. Generate both from the shared contract tree;
    // no private proto definitions are maintained by Keystone.
    let contracts_root = repo_root.join("capabilities");
    let proto_out = PathBuf::from(std::env::var("OUT_DIR")?);

    clean_generated_proto_dir(&proto_out)?;
    println!("cargo:rerun-if-changed={}", idl_root.display());
    println!("cargo:rerun-if-changed={}", contracts_root.display());
    println!("cargo:rerun-if-changed=build.rs");

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

    let protoc = protoc_bin_vendored::protoc_bin_path()?;
    // SAFETY: build scripts run single-threaded.
    unsafe {
        std::env::set_var("PROTOC", protoc);
    }
    let proto_files: Vec<PathBuf> = std::fs::read_dir(&proto_out)?
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.extension()
                .is_some_and(|extension| extension == "proto")
        })
        .collect();
    tonic_prost_build::configure()
        .build_server(true)
        .build_client(true)
        .compile_protos(&proto_files, std::slice::from_ref(&proto_out))?;
    Ok(())
}

/// Remove only generated proto/Rust files from this crate's Cargo output.
fn clean_generated_proto_dir(
    proto_out: &std::path::Path,
) -> Result<(), Box<dyn std::error::Error>> {
    if !proto_out.exists() {
        return Ok(());
    }
    for entry in std::fs::read_dir(proto_out)? {
        let path = entry?.path();
        if path
            .extension()
            .and_then(|extension| extension.to_str())
            .is_some_and(|extension| matches!(extension, "proto" | "rs"))
        {
            std::fs::remove_file(path)?;
        }
    }
    Ok(())
}
