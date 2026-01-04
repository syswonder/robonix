make build-sdk
eval $(make source-sdk)

make install
RUST_LOG=robonix_core=debug robonix-core