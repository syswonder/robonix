use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let proto_dir = manifest_dir.join("proto");
    let atlas_proto = proto_dir.join("atlas.proto");

    println!("cargo:rerun-if-changed={}", atlas_proto.display());

    let protoc = protoc_bin_vendored::protoc_bin_path()?;
    // SAFETY: build.rs is single-threaded.
    unsafe {
        std::env::set_var("PROTOC", protoc);
    }

    tonic_prost_build::configure()
        .build_server(true)
        .build_client(true)
        .compile_protos(&[atlas_proto], std::slice::from_ref(&proto_dir))?;

    Ok(())
}
