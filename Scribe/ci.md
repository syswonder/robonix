cargo fmt --all -- --check
cargo clippy --workspace --tests -- -D warnings
cargo build --workspace
cargo test --workspace --all-targets

