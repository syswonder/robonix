rm -rf tmp
mkdir -p tmp
cargo run -- --lang python --layout workspace \
    -I ../robonix-interfaces/lib/rcl_interfaces \
    -I ../robonix-interfaces/lib/common_interfaces \
    -o tmp \
    -i ../robonix-interfaces/ridl