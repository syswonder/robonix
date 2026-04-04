// SPDX-License-Identifier: MulanPSL-2.0
// build.rs — `robonix_contracts.proto` (+ imports) from robonix-codegen output.
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("robonix-pilot should live at rust/crates/robonix-pilot");

    let ridl_proto = workspace_root.join("crates/robonix-interfaces/robonix_proto");
    let contracts_proto = ridl_proto.join("robonix_contracts.proto");

    if !contracts_proto.is_file() {
        panic!(
            "missing generated proto — run: cargo run -p robonix-codegen -- --lang proto -I crates/robonix-interfaces/lib --contracts contracts -o crates/robonix-interfaces/robonix_proto",
        );
    }

    println!("cargo:rerun-if-changed={}", contracts_proto.display());

    let protoc = protoc_bin_vendored::protoc_bin_path()?;
    unsafe { std::env::set_var("PROTOC", protoc) };

    let google_inc = protoc_bin_vendored::include_path()?;
    let inc = vec![ridl_proto, google_inc];

    tonic_prost_build::configure()
        .build_server(true)
        .build_client(true)
        .compile_protos(&[contracts_proto], &inc)?;

    Ok(())
}
