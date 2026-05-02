use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .unwrap_or(&manifest_dir);
    let proto_dir = workspace_root.join("proto");
    let atlas_proto = proto_dir.join("atlas.proto");
    let legacy_proto = proto_dir.join("atlas_legacy.proto");

    println!("cargo:rerun-if-changed={}", atlas_proto.display());
    println!("cargo:rerun-if-changed={}", legacy_proto.display());

    let protoc = protoc_bin_vendored::protoc_bin_path()?;
    // SAFETY: build.rs is single-threaded.
    unsafe {
        std::env::set_var("PROTOC", protoc);
    }

    // Both services compiled into the same crate; legacy lives under its own
    // proto package (`robonix.runtime`) so symbols don't clash with the new
    // `robonix.atlas` types.
    tonic_prost_build::configure()
        .build_server(true)
        .build_client(true)
        .compile_protos(
            &[atlas_proto, legacy_proto],
            std::slice::from_ref(&proto_dir),
        )?;

    Ok(())
}
