// SPDX-License-Identifier: MulanPSL-2.0

#[cfg(target_os = "linux")]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    robonix_chronos::grpc::run().await
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("robonix-chronos requires Linux clock support");
    std::process::exit(1);
}
