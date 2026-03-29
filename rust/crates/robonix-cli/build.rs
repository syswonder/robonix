use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("robonix-cli should live at rust/crates/robonix-cli");

    let agent_chat_proto = workspace_root.join("proto/agent_chat.proto");
    if !agent_chat_proto.is_file() {
        eprintln!("cargo:warning=agent_chat.proto not found, skipping");
        return Ok(());
    }
    println!("cargo:rerun-if-changed={}", agent_chat_proto.display());

    let protoc = protoc_bin_vendored::protoc_bin_path()?;
    unsafe { std::env::set_var("PROTOC", protoc) };

    tonic_prost_build::configure()
        .build_server(false)
        .build_client(true)
        .compile_protos(&[agent_chat_proto], &[workspace_root.join("proto")])?;

    Ok(())
}
