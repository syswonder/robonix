# Pixel / data format identifiers for Robonix zero-copy buffer pipelines.
#
# These constants are shared across the control plane, buffer manager,
# and all node implementations. The integer values MUST match the
# robonix-buffer Rust crate (buffer.rs BufferFormat enum).
#
# When sensor_msgs/Image is used on a ROS topic, its string `encoding`
# field (e.g. "rgb8") is authoritative. When shared-memory transport is
# negotiated, this integer format tag is stored in the BufferHeader and
# in ZeroCopyFrame descriptors for O(1) matching without string compare.

# ── Image formats (0–10) ──
uint8 RGB8      = 0
uint8 BGR8      = 1
uint8 FLOAT32   = 2
uint8 NV12      = 3
uint8 DEPTH_U16 = 4
uint8 DEPTH_F32 = 5
uint8 MONO8     = 6
uint8 RGBA8     = 7
uint8 BGRA8     = 8
uint8 MONO16    = 9
uint8 F16       = 10

# ── Tensor / general formats (16+) ──
# Used for LLM embeddings, point clouds, voxel grids, etc.
uint8 BF16      = 16
uint8 INT8      = 17
uint8 INT32     = 18
uint8 INT64     = 19
uint8 FLOAT64   = 20

# Opaque raw bytes — consumer interprets via metadata_json shape/dtype.
uint8 RAW_BYTES = 255

uint8 format
