# config ip
sudo ip addr add 192.168.1.2/24 dev enx00e04c360241

# config can
sudo ip link set can2 up type can bitrate 500000

# config eth connection to raspi4b
sudo ip link set eno1 up
sudo ip addr add 192.168.100.1/24 dev eno1 # raspi4b will be at 192.168.100.2
