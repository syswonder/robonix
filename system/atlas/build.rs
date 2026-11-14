use std::path::PathBuf;

use robonix_codegen::codegen::{contract_gen, msg_parser, proto_gen};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let proto_dir = manifest_dir.join("proto");
    let atlas_proto = proto_dir.join("atlas.proto");

    println!("cargo:rerun-if-changed={}", atlas_proto.display());

    // Prefer a system protoc when present (needed on architectures like
    // LoongArch where protoc-bin-vendored does not ship a binary).
    let protoc = std::env::var_os("PROTOC")
        .map(PathBuf::from)
        .map(Ok)
        .unwrap_or_else(protoc_bin_vendored::protoc_bin_path)?;
    // SAFETY: build.rs is single-threaded.
    unsafe {
        std::env::set_var("PROTOC", protoc);
    }

    tonic_prost_build::configure()
        .build_server(true)
        .build_client(true)
        .compile_protos(&[atlas_proto], std::slice::from_ref(&proto_dir))?;

    // Business bindings are included only by the Channel integration test.
    let idl_root = manifest_dir.join("../../capabilities/lib");
    let contracts_root = manifest_dir.join("../../capabilities");
    let test_proto = PathBuf::from(std::env::var("OUT_DIR")?).join("channel-test");
    if test_proto.exists() {
        std::fs::remove_dir_all(&test_proto)?;
    }
    println!("cargo:rerun-if-changed={}", idl_root.display());
    println!("cargo:rerun-if-changed={}", contracts_root.display());

    let mut resolver = msg_parser::MsgResolver::new(std::slice::from_ref(&idl_root))?;
    let mut idl_skips = 0;
    resolver.resolve_all_in_index(false, &mut idl_skips)?;
    resolver.resolve_all_srv(false, &mut idl_skips)?;
    let contract_srvs = contract_gen::collect_referenced_srvs(&contracts_root)?;
    proto_gen::generate(&resolver, &test_proto, Some(&contract_srvs), false)?;
    contract_gen::generate(
        &mut resolver,
        std::slice::from_ref(&contracts_root),
        &test_proto,
        false,
    )?;
    tonic_prost_build::configure()
        .build_server(false)
        .compile_protos(
            &[test_proto.join("robonix_contracts.proto")],
            std::slice::from_ref(&test_proto),
        )?;

    Ok(())
}
