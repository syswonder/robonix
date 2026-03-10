cd ../ros2-client
echo PWD: $(pwd)
cargo run --bin msggen -- -i ../robonix-core/srv/Register.srv -o ../tools/generated/Register.rs

cargo run --bin msggen -- -i ../robonix-interfaces/lib/common_interfaces/sensor_msgs/msg/Image.msg -o ../tools/generated/Image.rs
cargo run --bin msggen -- -i ../robonix-interfaces/lib/common_interfaces/sensor_msgs/msg/PointCloud2.msg -o ../tools/generated/PointCloud2.rs