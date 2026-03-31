# General-purpose buffer descriptor for Robonix managed buffers.
#
# While ZeroCopyFrame is specialized for image/video data with
# width/height/channels semantics, BufferDescriptor covers arbitrary
# typed buffers: tensors, point clouds, joint state arrays, voxel
# grids, or any contiguous data region managed by the buffer system.

uint64 handle_id

# POSIX SHM name assigned by the control plane.
string shm_name

# N-dimensional shape (e.g. [1080, 1920, 3] for an image,
# [1, 3, 480, 640] for a NCHW tensor, [N, 6] for XYZRGB points).
uint32[] shape

# Byte strides per dimension. Length matches shape.
uint32[] strides

# Total data payload size in bytes.
uint64 size_bytes

# Data element format.
robonix_msg/BufferFormat format

# Memory domain.
robonix_msg/MemoryDomain memory_domain

# Hardware timestamp (nanoseconds since UNIX epoch).
uint64 timestamp_ns

# Monotonic sequence counter.
uint64 sequence

# Sync token (CUDA event, NvSciSync, or 0).
uint64 sync_token

# Pool identifier (0 if not pooled).
uint32 pool_id

# Whether pages are GPU-pinned.
bool gpu_pinned
