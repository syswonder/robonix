// SPDX-License-Identifier: MulanPSL-2.0
// build.rs — protos only from ridlc output (`executor.proto` imports `pilot.proto`).
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("robonix-executor should live at rust/crates/robonix-executor");
    let ridl_proto = workspace_root.join("robonix-interfaces/robonix_proto");
    let pilot_proto = ridl_proto.join("pilot.proto");
    let executor_proto = ridl_proto.join("executor.proto");

    println!("cargo:rerun-if-changed={}", pilot_proto.display());
    println!("cargo:rerun-if-changed={}", executor_proto.display());

    let protoc = protoc_bin_vendored::protoc_bin_path()?;
    unsafe { std::env::set_var("PROTOC", protoc) };

    let inc = vec![ridl_proto];
    tonic_prost_build::configure()
        .build_server(true)
        .build_client(true)
        .compile_protos(&[pilot_proto, executor_proto], &inc)?;

    Ok(())
}
