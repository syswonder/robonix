//! Compile VLM + robonix_msg protos from `robonix-interfaces/robonix_proto` (pre-generated).
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("robonix-agent should live at rust/crates/robonix-agent");
    let proto_root = workspace_root.join("robonix-interfaces/robonix_proto");
    let vlm_proto = proto_root.join("vlm.proto");

    if !vlm_proto.is_file() {
        panic!(
            "missing {} — regenerate with: ridlc --lang proto -I robonix-interfaces/lib -o robonix-interfaces/robonix_proto",
            vlm_proto.display()
        );
    }

    println!("cargo:rerun-if-changed={}", vlm_proto.display());
    for entry in std::fs::read_dir(&proto_root)? {
        let entry = entry?;
        let p = entry.path();
        if p.extension().and_then(|s| s.to_str()) == Some("proto") {
            println!("cargo:rerun-if-changed={}", p.display());
        }
    }

    let protoc = protoc_bin_vendored::protoc_bin_path()?;
    unsafe { std::env::set_var("PROTOC", protoc) };

    tonic_prost_build::configure()
        .build_server(false)
        .build_client(true)
        .compile_protos(&[vlm_proto], &[proto_root])?;

    Ok(())
}
