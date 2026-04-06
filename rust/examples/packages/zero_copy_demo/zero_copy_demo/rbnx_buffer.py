"""Python bindings for librobonix_buffer.so (Rust C FFI).

This module loads the Robonix buffer manager shared library and exposes
a Pythonic API. All buffer allocation, GPU pinning, and lifecycle
management is performed by the Rust library — Python nodes only receive
pointers and numpy views.
"""
from __future__ import annotations

import ctypes
import os
from pathlib import Path
from typing import Optional

import numpy as np

# Locate the shared library: check env var, then relative build paths.
def _find_lib() -> str:
    if p := os.environ.get("ROBONIX_BUFFER_LIB"):
        return p
    # zero_copy_demo/ -> packages/ -> examples/ -> rust/
    rust_root = Path(__file__).resolve().parents[4]
    for mode in ("release", "debug"):
        candidate = rust_root / "target" / mode / "librobonix_buffer.so"
        if candidate.exists():
            return str(candidate)
    raise RuntimeError(
        f"librobonix_buffer.so not found (searched {rust_root}/target/{{release,debug}}/).\n"
        "Build with: cd rust && cargo build -p robonix-buffer\n"
        "Or set ROBONIX_BUFFER_LIB=/path/to/librobonix_buffer.so"
    )


_lib: Optional[ctypes.CDLL] = None


def _get_lib() -> ctypes.CDLL:
    global _lib
    if _lib is None:
        path = _find_lib()
        _lib = ctypes.CDLL(path)

        # buffer manager
        _lib.rbnx_buf_new.restype = ctypes.c_void_p
        _lib.rbnx_buf_destroy.argtypes = [ctypes.c_void_p]
        _lib.rbnx_buf_cuda_available.argtypes = [ctypes.c_void_p]
        _lib.rbnx_buf_cuda_available.restype = ctypes.c_int
        _lib.rbnx_buf_allocate.argtypes = [
            ctypes.c_void_p, ctypes.c_char_p,
            ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
        ]
        _lib.rbnx_buf_allocate.restype = ctypes.c_uint64
        _lib.rbnx_buf_data_ptr.argtypes = [ctypes.c_void_p, ctypes.c_uint64]
        _lib.rbnx_buf_data_ptr.restype = ctypes.c_void_p
        _lib.rbnx_buf_header_ptr.argtypes = [ctypes.c_void_p, ctypes.c_uint64]
        _lib.rbnx_buf_header_ptr.restype = ctypes.c_void_p
        _lib.rbnx_buf_data_len.argtypes = [ctypes.c_void_p, ctypes.c_uint64]
        _lib.rbnx_buf_data_len.restype = ctypes.c_size_t
        _lib.rbnx_buf_attach.argtypes = [ctypes.c_void_p, ctypes.c_uint64, ctypes.c_int]
        _lib.rbnx_buf_attach.restype = ctypes.c_int
        _lib.rbnx_buf_signal_write.argtypes = [ctypes.c_void_p, ctypes.c_uint64]
        _lib.rbnx_buf_read_seq.argtypes = [ctypes.c_void_p, ctypes.c_uint64]
        _lib.rbnx_buf_read_seq.restype = ctypes.c_uint64
        _lib.rbnx_buf_is_pinned.argtypes = [ctypes.c_void_p, ctypes.c_uint64]
        _lib.rbnx_buf_is_pinned.restype = ctypes.c_int
        _lib.rbnx_buf_detach.argtypes = [ctypes.c_void_p, ctypes.c_uint64]
        _lib.rbnx_buf_detach.restype = ctypes.c_int
        _lib.rbnx_buf_release.argtypes = [ctypes.c_void_p, ctypes.c_uint64]
        _lib.rbnx_buf_release.restype = ctypes.c_int

        # raw-bytes allocate (for tensors, point clouds, etc.)
        _lib.rbnx_buf_allocate_raw.argtypes = [
            ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t, ctypes.c_uint32,
        ]
        _lib.rbnx_buf_allocate_raw.restype = ctypes.c_uint64

        # cross-process open
        _lib.rbnx_buf_open.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        _lib.rbnx_buf_open.restype = ctypes.c_uint64

        # CUDA helpers
        _lib.rbnx_cuda_available.restype = ctypes.c_int
        _lib.rbnx_cuda_device_malloc.argtypes = [ctypes.c_size_t]
        _lib.rbnx_cuda_device_malloc.restype = ctypes.c_void_p
        _lib.rbnx_cuda_device_free.argtypes = [ctypes.c_void_p]
        _lib.rbnx_cuda_device_free.restype = ctypes.c_int
        _lib.rbnx_cuda_memcpy_h2d.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t]
        _lib.rbnx_cuda_memcpy_h2d.restype = ctypes.c_int
        _lib.rbnx_cuda_memcpy_d2h.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t]
        _lib.rbnx_cuda_memcpy_d2h.restype = ctypes.c_int
        _lib.rbnx_cuda_memcpy_d2d.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t]
        _lib.rbnx_cuda_memcpy_d2d.restype = ctypes.c_int
        _lib.rbnx_cuda_device_sync.restype = ctypes.c_int

        # CUDA IPC
        _lib.rbnx_cuda_ipc_get_handle.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        _lib.rbnx_cuda_ipc_get_handle.restype = ctypes.c_int
        _lib.rbnx_cuda_ipc_open_handle.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p)]
        _lib.rbnx_cuda_ipc_open_handle.restype = ctypes.c_int
        _lib.rbnx_cuda_ipc_close_handle.argtypes = [ctypes.c_void_p]
        _lib.rbnx_cuda_ipc_close_handle.restype = ctypes.c_int
    return _lib


# Buffer format constants — values MUST match robonix_msg/msg/BufferFormat.msg
# and the Rust BufferFormat enum in robonix-buffer/src/buffer.rs.
#
# Image formats (0–10)
FORMAT_RGB8      = 0
FORMAT_BGR8      = 1
FORMAT_FLOAT32   = 2
FORMAT_NV12      = 3
FORMAT_DEPTH_U16 = 4
FORMAT_DEPTH_F32 = 5
FORMAT_MONO8     = 6
FORMAT_RGBA8     = 7
FORMAT_BGRA8     = 8
FORMAT_MONO16    = 9
FORMAT_F16       = 10
# Tensor / general formats (16+)
FORMAT_BF16      = 16
FORMAT_INT8      = 17
FORMAT_INT32     = 18
FORMAT_INT64     = 19
FORMAT_FLOAT64   = 20
FORMAT_RAW_BYTES = 255

# Mapping from BufferFormat integer to numpy dtype and ROS 2 encoding string.
_FORMAT_META = {
    FORMAT_RGB8:      {"dtype": "uint8",   "bpe": 1, "ros_encoding": "rgb8"},
    FORMAT_BGR8:      {"dtype": "uint8",   "bpe": 1, "ros_encoding": "bgr8"},
    FORMAT_FLOAT32:   {"dtype": "float32", "bpe": 4, "ros_encoding": "32FC1"},
    FORMAT_NV12:      {"dtype": "uint8",   "bpe": 1, "ros_encoding": "nv12"},
    FORMAT_DEPTH_U16: {"dtype": "uint16",  "bpe": 2, "ros_encoding": "16UC1"},
    FORMAT_DEPTH_F32: {"dtype": "float32", "bpe": 4, "ros_encoding": "32FC1"},
    FORMAT_MONO8:     {"dtype": "uint8",   "bpe": 1, "ros_encoding": "mono8"},
    FORMAT_RGBA8:     {"dtype": "uint8",   "bpe": 1, "ros_encoding": "rgba8"},
    FORMAT_BGRA8:     {"dtype": "uint8",   "bpe": 1, "ros_encoding": "bgra8"},
    FORMAT_MONO16:    {"dtype": "uint16",  "bpe": 2, "ros_encoding": "mono16"},
    FORMAT_F16:       {"dtype": "float16", "bpe": 2, "ros_encoding": "16FC1"},
    FORMAT_BF16:      {"dtype": "bfloat16","bpe": 2, "ros_encoding": "bf16"},
    FORMAT_INT8:      {"dtype": "int8",    "bpe": 1, "ros_encoding": "int8"},
    FORMAT_INT32:     {"dtype": "int32",   "bpe": 4, "ros_encoding": "int32"},
    FORMAT_INT64:     {"dtype": "int64",   "bpe": 8, "ros_encoding": "int64"},
    FORMAT_FLOAT64:   {"dtype": "float64", "bpe": 8, "ros_encoding": "float64"},
    FORMAT_RAW_BYTES: {"dtype": "uint8",   "bpe": 1, "ros_encoding": "raw"},
}


def format_dtype(fmt: int) -> str:
    """Return the numpy dtype string for a BufferFormat value."""
    return _FORMAT_META.get(fmt, _FORMAT_META[FORMAT_RGB8])["dtype"]


class BufferHandle:
    """A view into a Robonix-managed buffer.

    Provides numpy access to the shared memory data region. The Rust
    library owns the underlying SHM; this is just a typed view.

    The ``format`` field uses values from ``robonix_msg/msg/BufferFormat.msg``.
    """

    def __init__(
        self,
        mgr_ptr: int,
        handle_id: int,
        width: int,
        height: int,
        channels: int,
        fmt: int = FORMAT_RGB8,
    ):
        self._mgr = mgr_ptr
        self.handle_id = handle_id
        self.width = width
        self.height = height
        self.channels = channels
        self.format = fmt
        lib = _get_lib()
        self.data_ptr = lib.rbnx_buf_data_ptr(mgr_ptr, handle_id)
        self.data_len = lib.rbnx_buf_data_len(mgr_ptr, handle_id)

    def as_numpy(self) -> np.ndarray:
        """Return a numpy view of the data region. No copy.

        The dtype is determined by the BufferFormat (e.g. uint8 for RGB8,
        float32 for FLOAT32, uint16 for DEPTH_U16).
        """
        dt = np.dtype(format_dtype(self.format))
        n_elements = self.data_len // dt.itemsize
        arr = (ctypes.c_uint8 * self.data_len).from_address(self.data_ptr)
        return np.ctypeslib.as_array(arr).view(dt)[:n_elements].reshape(
            self.height, self.width, self.channels
        )

    @property
    def is_pinned(self) -> bool:
        return _get_lib().rbnx_buf_is_pinned(self._mgr, self.handle_id) != 0

    @property
    def seq(self) -> int:
        return _get_lib().rbnx_buf_read_seq(self._mgr, self.handle_id)


class RobonixBufferManager:
    """Python interface to the Rust RobonixBufferManager.

    This is the system-level buffer coordinator. All buffer allocation,
    GPU pinning, and lifecycle management goes through this class.
    """

    def __init__(self):
        self._ptr = None
        lib = _get_lib()
        self._ptr = lib.rbnx_buf_new()
        if not self._ptr:
            raise RuntimeError("Failed to create RobonixBufferManager")

    @property
    def cuda_available(self) -> bool:
        return _get_lib().rbnx_buf_cuda_available(self._ptr) != 0

    def allocate(
        self, shm_name: str, width: int, height: int, channels: int = 3, fmt: int = FORMAT_RGB8
    ) -> BufferHandle:
        """Allocate a system-managed buffer backed by POSIX shared memory.

        ``fmt`` must be a value from ``robonix_msg/msg/BufferFormat.msg``
        (e.g. ``FORMAT_RGB8``, ``FORMAT_DEPTH_U16``).
        """
        lib = _get_lib()
        handle_id = lib.rbnx_buf_allocate(
            self._ptr, shm_name.encode("utf-8"), width, height, channels, fmt,
        )
        if handle_id == 0:
            raise RuntimeError(f"Failed to allocate buffer: {shm_name}")
        return BufferHandle(self._ptr, handle_id, width, height, channels, fmt)

    def open(self, shm_name: str, pin_for_gpu: bool = False) -> BufferHandle:
        """Open an existing SHM buffer created by another process (consumer-side).

        The Rust side reads the BufferHeader to discover geometry
        automatically — no need to pass width/height/channels.
        """
        lib = _get_lib()
        handle_id = lib.rbnx_buf_open(self._ptr, shm_name.encode("utf-8"))
        if handle_id == 0:
            raise RuntimeError(f"Failed to open buffer: {shm_name}")
        if pin_for_gpu:
            rc = lib.rbnx_buf_attach(self._ptr, handle_id, 1)
            if rc != 0:
                raise RuntimeError(f"Failed to GPU-pin opened buffer: {shm_name}")
        header_ptr = lib.rbnx_buf_header_ptr(self._ptr, handle_id)
        if not header_ptr:
            raise RuntimeError(f"Failed to read buffer header: {shm_name}")
        width = ctypes.c_uint32.from_address(header_ptr + 24).value
        height = ctypes.c_uint32.from_address(header_ptr + 28).value
        channels = ctypes.c_uint32.from_address(header_ptr + 32).value
        fmt = ctypes.c_uint32.from_address(header_ptr + 40).value
        return BufferHandle(self._ptr, handle_id, width, height, channels, fmt)

    def attach(self, handle_id: int, pin_for_gpu: bool = False) -> BufferHandle:
        """Attach a consumer to a buffer already known to this manager.

        For cross-process attach, use ``open()`` instead.
        """
        lib = _get_lib()
        rc = lib.rbnx_buf_attach(self._ptr, handle_id, int(pin_for_gpu))
        if rc != 0:
            raise RuntimeError(f"Failed to attach to buffer handle={handle_id}")
        header_ptr = lib.rbnx_buf_header_ptr(self._ptr, handle_id)
        if not header_ptr:
            raise RuntimeError(f"Failed to read buffer header for handle={handle_id}")
        width = ctypes.c_uint32.from_address(header_ptr + 24).value
        height = ctypes.c_uint32.from_address(header_ptr + 28).value
        channels = ctypes.c_uint32.from_address(header_ptr + 32).value
        fmt = ctypes.c_uint32.from_address(header_ptr + 40).value
        return BufferHandle(self._ptr, handle_id, width, height, channels, fmt)

    def detach(self, handle_id: int):
        _get_lib().rbnx_buf_detach(self._ptr, handle_id)

    def release(self, handle_id: int):
        _get_lib().rbnx_buf_release(self._ptr, handle_id)

    def destroy(self):
        if self._ptr:
            _get_lib().rbnx_buf_destroy(self._ptr)
            self._ptr = None

    def __del__(self):
        self.destroy()


# ── CUDA helpers (direct FFI) ──

def cuda_device_malloc(size: int) -> int:
    ptr = _get_lib().rbnx_cuda_device_malloc(size)
    if not ptr:
        raise RuntimeError("cudaMalloc failed")
    return ptr


def cuda_device_free(ptr: int):
    _get_lib().rbnx_cuda_device_free(ptr)


def cuda_memcpy_d2d(dst_device: int, src_device: int, size: int):
    """Device-to-device copy (no CPU or PCIe involved)."""
    rc = _get_lib().rbnx_cuda_memcpy_d2d(dst_device, src_device, size)
    if rc != 0:
        raise RuntimeError("cudaMemcpy D2D failed")


def cuda_device_sync():
    _get_lib().rbnx_cuda_device_sync()


# ── CUDA IPC helpers ──

IPC_HANDLE_SIZE = 64


def cuda_ipc_get_handle(dev_ptr: int) -> bytes:
    """Export a device pointer as a 64-byte IPC handle.

    The returned bytes can be written to SHM or sent via gRPC metadata
    so another process can import the same GPU memory region.
    """
    handle_buf = (ctypes.c_uint8 * IPC_HANDLE_SIZE)()
    rc = _get_lib().rbnx_cuda_ipc_get_handle(dev_ptr, handle_buf)
    if rc != 0:
        raise RuntimeError("cudaIpcGetMemHandle failed")
    return bytes(handle_buf)


def cuda_ipc_open_handle(handle: bytes) -> int:
    """Import a device pointer from a 64-byte IPC handle (consumer side).

    Returns a device pointer valid in this process's CUDA context.
    Must be released with ``cuda_ipc_close_handle``.
    """
    if len(handle) != IPC_HANDLE_SIZE:
        raise ValueError(f"IPC handle must be {IPC_HANDLE_SIZE} bytes, got {len(handle)}")
    handle_buf = (ctypes.c_uint8 * IPC_HANDLE_SIZE)(*handle)
    ptr = ctypes.c_void_p()
    rc = _get_lib().rbnx_cuda_ipc_open_handle(handle_buf, ctypes.byref(ptr))
    if rc != 0:
        raise RuntimeError("cudaIpcOpenMemHandle failed")
    return ptr.value


def cuda_ipc_close_handle(dev_ptr: int):
    """Release a device pointer obtained via ``cuda_ipc_open_handle``."""
    rc = _get_lib().rbnx_cuda_ipc_close_handle(dev_ptr)
    if rc != 0:
        raise RuntimeError("cudaIpcCloseMemHandle failed")
