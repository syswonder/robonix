# Zero-copy frame descriptor for Robonix shared-memory transport.
#
# This message replaces sensor_msgs/Image on the shared-memory data
# plane. Instead of carrying pixel data inline (which forces a copy
# on every publish/subscribe), it carries a *reference* to a buffer
# that has already been allocated and managed by the Robonix buffer
# manager.
#
# Data flow:
#   Producer → writes pixels into Robonix-managed SHM buffer
#   Producer → publishes ZeroCopyFrame descriptor (tiny, ~128 bytes)
#   Consumer → receives descriptor, maps the SHM region by handle
#   Consumer → reads pixels at data offset — zero copies
#
# On ROS 2 topics this message is CDR-serialized like any other;
# the expensive pixel data is NOT in the message — only the handle.
# On gRPC, serialized as protobuf (same small descriptor).

std_msgs/Header header

# Robonix buffer manager handle — globally unique within the system.
uint64 buffer_handle_id

# POSIX SHM name (e.g. "/rbnx_shm_<uuid>") assigned by the control
# plane via NegotiateChannel.
string shm_name

# Frame geometry
uint32 width
uint32 height
uint32 channels
uint32 stride

# Pixel format — values from BufferFormat.msg
robonix_msg/BufferFormat format

# Memory domain — values from MemoryDomain.msg
robonix_msg/MemoryDomain memory_domain

# Total data payload size in bytes (stride * height for image data)
uint64 data_size

# Monotonically increasing sequence number set by the producer.
# Consumers can detect dropped frames by checking for gaps.
uint64 sequence

# Hardware capture timestamp in nanoseconds since UNIX epoch.
# Distinct from header.stamp which may be set at publish time.
uint64 hw_timestamp_ns

# Synchronization token. Interpretation depends on the memory domain:
#   CPU:     0 (no sync needed — producer/consumer use write_seq/read_ack)
#   GPU:     CUDA event handle for cross-stream sync
#   Unified: NvSciSync token on Jetson
uint64 sync_token

# Whether the underlying SHM pages are registered with CUDA for
# pinned DMA transfers (cudaHostRegister).
bool gpu_pinned

# Buffer pool identifier (0 if not using pool allocation).
uint32 pool_id
