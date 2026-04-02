// SPDX-License-Identifier: MulanPSL-2.0
// build.rs — protos only from `robonix-interfaces/robonix_proto` (ridlc output).
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("robonix-pilot should live at rust/crates/robonix-pilot");

    let ridl_proto = workspace_root.join("robonix-interfaces/robonix_proto");

    let pilot_proto = ridl_proto.join("pilot.proto");
    let executor_proto = ridl_proto.join("executor.proto");
    let vlm_proto = ridl_proto.join("vlm.proto");

    if !pilot_proto.is_file() || !vlm_proto.is_file() {
        panic!(
            "missing generated proto — run: cargo run -p ridlc -- --lang proto -I robonix-interfaces/lib --contracts contracts -o robonix-interfaces/robonix_proto",
        );
    }

    for p in [&pilot_proto, &executor_proto, &vlm_proto] {
        println!("cargo:rerun-if-changed={}", p.display());
    }

    let protoc = protoc_bin_vendored::protoc_bin_path()?;
    unsafe { std::env::set_var("PROTOC", protoc) };

    let proto_inc = vec![ridl_proto.clone()];
    tonic_prost_build::configure()
        .build_server(true)
        .build_client(true)
        .compile_protos(&[pilot_proto, executor_proto], &proto_inc)?;

    tonic_prost_build::configure()
        .build_server(false)
        .build_client(true)
        .compile_protos(&[vlm_proto], &[ridl_proto])?;

    Ok(())
}
