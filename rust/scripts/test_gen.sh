cd ../ros2-client
echo PWD: $(pwd)
cargo run --bin msggen -- -i ../robonix-core/srv/Register.srv -o ../scripts/generated/Register.rs