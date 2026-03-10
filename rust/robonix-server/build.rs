fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("cargo:rerun-if-changed=../ridlc/proto/robonix_runtime.proto");

    tonic_prost_build::configure()
        .build_server(true)
        .build_client(false)
        .compile_protos(
            &["../ridlc/proto/robonix_runtime.proto"],
            &["../ridlc/proto"],
        )?;

    Ok(())
}
