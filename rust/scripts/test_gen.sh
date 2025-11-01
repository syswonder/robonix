cd ../ros2-client
echo PWD: $(pwd)
cargo run --bin msggen -- -i ../robonix-core/srv/Register.srv -o ../scripts/generated/Register.rs

cargo run --bin msggen -- -i ../common_interfaces/sensor_msgs/msg/Image.msg -o ../scripts/generated/Image.rs
cargo run --bin msggen -- -i ../common_interfaces/sensor_msgs/msg/PointCloud2.msg -o ../scripts/generated/PointCloud2.rs