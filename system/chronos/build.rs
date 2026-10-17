// SPDX-License-Identifier: MulanPSL-2.0

use std::path::PathBuf;

use robonix_codegen::codegen::{contract_gen, msg_parser, proto_gen};

/// Generate only Chronos and shared lifecycle protobuf/tonic bindings.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let idl_root = root.join("capabilities/lib");
    let contracts = root.join("capabilities/system/chronos");
    let driver = root.join("capabilities/lifecycle/driver.v1.toml");
    let out = PathBuf::from(std::env::var("OUT_DIR")?);
    let selected = out.join("chronos_contracts");
    std::fs::create_dir_all(&selected)?;
    for name in [
        "quality.v1.toml",
        "reference_time.v1.toml",
        "conversion_models.v1.toml",
        "observe_device_timestamp.v1.toml",
    ] {
        std::fs::copy(contracts.join(name), selected.join(name))?;
    }
    std::fs::copy(&driver, selected.join("driver.v1.toml"))?;
    println!("cargo:rerun-if-changed={}", contracts.display());
    println!("cargo:rerun-if-changed={}", driver.display());
    println!("cargo:rerun-if-changed={}", idl_root.display());
    println!("cargo:rerun-if-changed=build.rs");
    let mut resolver = msg_parser::MsgResolver::new(std::slice::from_ref(&idl_root))?;
    // Resolving the selected messages recursively avoids requiring unrelated IDL.
    for name in ["ClockQualitySnapshot", "ClockModelsSnapshot"] {
        resolver.resolve_named_type("chronos", name, None)?;
    }
    let srvs = contract_gen::collect_referenced_srvs(&selected)?;
    for (package, name) in &srvs {
        resolver.resolve_srv(package, name)?;
    }
    proto_gen::generate(&resolver, &out, Some(&srvs), false)?;
    contract_gen::generate(&mut resolver, &[selected], &out, false)?;
    let protoc = std::env::var_os("PROTOC")
        .map(PathBuf::from)
        .map(Ok)
        .unwrap_or_else(protoc_bin_vendored::protoc_bin_path)?;
    // SAFETY: this build script is single-threaded before invoking protoc.
    unsafe { std::env::set_var("PROTOC", protoc) };
    tonic_prost_build::configure().compile_protos(
        &[out.join("robonix_contracts.proto")],
        std::slice::from_ref(&out),
    )?;
    Ok(())
}
