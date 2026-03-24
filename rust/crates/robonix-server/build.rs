use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .unwrap_or(&manifest_dir);
    let proto_dir = workspace_root.join("proto");

    println!(
        "cargo:rerun-if-changed={}",
        proto_dir.join("robonix_runtime.proto").display()
    );

    let protoc = protoc_bin_vendored::protoc_bin_path()?;
    unsafe {
        std::env::set_var("PROTOC", protoc);
    }

    tonic_prost_build::configure()
        .build_server(true)
        .build_client(true)
        .compile_protos(
            &[proto_dir.join("robonix_runtime.proto")],
            &[proto_dir.clone()],
        )?;

    // RIDL codegen REMOVED -- RIDL syntax is deprecated.
    // See robonix-interfaces/ridl/DEPRECATED.md

    Ok(())
}
