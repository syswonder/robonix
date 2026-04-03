// SPDX-License-Identifier: MulanPSL-2.0
// build.rs — `liaison.proto` + `pilot.proto` (client to Pilot) from robonix-codegen output only.
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("robonix-liaison should live at rust/crates/robonix-liaison");
    let ridl_proto = workspace_root.join("crates/robonix-interfaces/robonix_proto");
    let liaison_proto = ridl_proto.join("liaison.proto");
    let pilot_proto = ridl_proto.join("pilot.proto");

    println!("cargo:rerun-if-changed={}", liaison_proto.display());
    println!("cargo:rerun-if-changed={}", pilot_proto.display());

    let protoc = protoc_bin_vendored::protoc_bin_path()?;
    unsafe { std::env::set_var("PROTOC", protoc) };

    let inc = vec![ridl_proto];
    tonic_prost_build::configure()
        .build_server(true)
        .build_client(true)
        .compile_protos(&[liaison_proto, pilot_proto], &inc)?;

    Ok(())
}
