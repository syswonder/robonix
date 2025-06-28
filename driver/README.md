# Driver

The hardware abstraction layer, responsible for transforming physical device interfaces into software-accessible data streams. Supports high-performance, low-latency communication via:

- **ROS 2 Topics** (standard interface)
- **Zero-copy IPC** (shared memory, DMA-BUF)
- **Sensor abstraction** (e.g., RGB/Depth cameras, IMU, Lidar, CAN devices)

This layer enables upper modules to ignore hardware details and focus on functional logic.
