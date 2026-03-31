//! # robonix-buffer
//!
//! System-level buffer management for the Robonix embodied intelligence OS.
//!
//! This crate is the **single owner** of all shared memory buffers in a
//! Robonix system. Nodes never allocate or manage buffers directly — they
//! request access through the [`RobonixBufferManager`], which:
//!
//! 1. Allocates POSIX SHM regions with a fixed [`BufferHeader`].
//! 2. Pins pages for GPU DMA via `cudaHostRegister` (discrete GPU) or
//!    provides direct device pointers (Jetson UMA — TODO).
//! 3. Tracks consumers via reference counting and manages lifecycle.
//! 4. Exposes a C FFI so Python / other languages can use the same manager.

pub mod buffer;
pub mod cuda;
pub mod error;
pub mod manager;
pub mod shm;

pub use buffer::{BufferFormat, BufferHeader, BufferSpec, BufferState, HEADER_SIZE, MemoryDomain};
pub use cuda::CudaIpcMemHandle;
pub use error::{BufferError, Result};
pub use manager::{BufferView, RobonixBufferManager};
pub use shm::ShmRegion;

// ─── C FFI ───────────────────────────────────────────────────────────
//
// Exposes the buffer manager to Python (via ctypes) and C/C++ consumers.
// The Python demo loads `librobonix_buffer.so` and calls these functions.

use std::ffi::{CStr, c_char, c_int};

/// Create a new buffer manager. Returns an opaque pointer.
/// The caller must eventually call `rbnx_buf_destroy` to free it.
#[unsafe(no_mangle)]
pub extern "C" fn rbnx_buf_new() -> *mut RobonixBufferManager {
    let _ = env_logger::try_init();
    Box::into_raw(Box::new(RobonixBufferManager::new()))
}

/// Destroy a buffer manager (releases all buffers).
///
/// # Safety
/// `mgr` must be a valid pointer from `rbnx_buf_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_buf_destroy(mgr: *mut RobonixBufferManager) {
    if !mgr.is_null() {
        unsafe {
            drop(Box::from_raw(mgr));
        }
    }
}

/// Check if CUDA is available.
///
/// # Safety
/// `mgr` must be a valid pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_buf_cuda_available(mgr: *const RobonixBufferManager) -> c_int {
    unsafe { (*mgr).cuda_available() as c_int }
}

/// Allocate a buffer. Returns the handle ID (>0) or 0 on error.
///
/// # Safety
/// `mgr` and `shm_name` must be valid pointers.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_buf_allocate(
    mgr: *mut RobonixBufferManager,
    shm_name: *const c_char,
    width: u32,
    height: u32,
    channels: u32,
    format: u32,
) -> u64 {
    let name = unsafe { CStr::from_ptr(shm_name) }.to_string_lossy();
    let fmt = match BufferFormat::from_u32(format) {
        Some(f) => f,
        None => return 0,
    };
    let spec = BufferSpec {
        width,
        height,
        channels,
        format: fmt,
    };
    unsafe { (*mgr).allocate(&name, spec).unwrap_or(0) }
}

/// Get the raw data pointer for a buffer (header + data region start).
/// Returns null on error.
///
/// # Safety
/// `mgr` must be valid and `handle_id` must refer to an allocated buffer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_buf_data_ptr(
    mgr: *const RobonixBufferManager,
    handle_id: u64,
) -> *mut u8 {
    match unsafe { (*mgr).producer_view(handle_id) } {
        Ok(view) => view.data_ptr,
        Err(_) => std::ptr::null_mut(),
    }
}

/// Get the header pointer for a buffer.
///
/// # Safety
/// `mgr` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_buf_header_ptr(
    mgr: *const RobonixBufferManager,
    handle_id: u64,
) -> *const u8 {
    match unsafe { (*mgr).producer_view(handle_id) } {
        Ok(view) => view.header_ptr,
        Err(_) => std::ptr::null(),
    }
}

/// Get the data region size in bytes.
///
/// # Safety
/// `mgr` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_buf_data_len(
    mgr: *const RobonixBufferManager,
    handle_id: u64,
) -> usize {
    match unsafe { (*mgr).producer_view(handle_id) } {
        Ok(view) => view.data_len,
        Err(_) => 0,
    }
}

/// Allocate a raw-bytes buffer (for tensors, point clouds, embeddings, etc.).
/// Shape/stride information is communicated via metadata_json, not the header.
/// Returns the handle ID (>0) or 0 on error.
///
/// # Safety
/// `mgr` and `shm_name` must be valid pointers.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_buf_allocate_raw(
    mgr: *mut RobonixBufferManager,
    shm_name: *const c_char,
    data_bytes: usize,
    format: u32,
) -> u64 {
    let name = unsafe { CStr::from_ptr(shm_name) }.to_string_lossy();
    let fmt = match BufferFormat::from_u32(format) {
        Some(f) => f,
        None => return 0,
    };
    unsafe { (*mgr).allocate_raw(&name, data_bytes, fmt).unwrap_or(0) }
}

/// Open an existing buffer created by another process (consumer-side).
/// Reads the BufferHeader from SHM to discover geometry automatically.
/// Returns the handle ID (>0) or 0 on error.
///
/// # Safety
/// `mgr` and `shm_name` must be valid pointers.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_buf_open(
    mgr: *mut RobonixBufferManager,
    shm_name: *const c_char,
) -> u64 {
    let name = unsafe { CStr::from_ptr(shm_name) }.to_string_lossy();
    unsafe { (*mgr).open(&name).unwrap_or(0) }
}

/// Attach a consumer. If `pin_gpu` != 0, pins for GPU DMA.
/// Returns 0 on success, -1 on error.
///
/// # Safety
/// `mgr` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_buf_attach(
    mgr: *mut RobonixBufferManager,
    handle_id: u64,
    pin_gpu: c_int,
) -> c_int {
    match unsafe { (*mgr).attach(handle_id, pin_gpu != 0) } {
        Ok(_) => 0,
        Err(_) => -1,
    }
}

/// Signal that the producer has written a new frame.
///
/// # Safety
/// `mgr` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_buf_signal_write(mgr: *const RobonixBufferManager, handle_id: u64) {
    if let Ok(view) = unsafe { (*mgr).producer_view(handle_id) } {
        view.header().signal_write();
    }
}

/// Get the current write sequence number.
///
/// # Safety
/// `mgr` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_buf_read_seq(
    mgr: *const RobonixBufferManager,
    handle_id: u64,
) -> u64 {
    match unsafe { (*mgr).producer_view(handle_id) } {
        Ok(view) => view.header().current_seq(),
        Err(_) => 0,
    }
}

/// Check if a buffer is GPU-pinned.
///
/// # Safety
/// `mgr` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_buf_is_pinned(
    mgr: *const RobonixBufferManager,
    handle_id: u64,
) -> c_int {
    match unsafe { (*mgr).producer_view(handle_id) } {
        Ok(view) => view.gpu_pinned as c_int,
        Err(_) => 0,
    }
}

/// Detach a consumer from a buffer.
///
/// # Safety
/// `mgr` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_buf_detach(mgr: *mut RobonixBufferManager, handle_id: u64) -> c_int {
    match unsafe { (*mgr).detach(handle_id) } {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

/// Release a buffer (unmaps SHM, unlinks name).
///
/// # Safety
/// `mgr` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_buf_release(mgr: *mut RobonixBufferManager, handle_id: u64) -> c_int {
    match unsafe { (*mgr).release(handle_id) } {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

// ─── CUDA convenience wrappers (C FFI) ──────────────────────────────

#[unsafe(no_mangle)]
pub extern "C" fn rbnx_cuda_available() -> c_int {
    cuda::cuda_available() as c_int
}

#[unsafe(no_mangle)]
pub extern "C" fn rbnx_cuda_device_malloc(size: usize) -> *mut u8 {
    cuda::device_malloc(size).unwrap_or(std::ptr::null_mut())
}

/// # Safety
/// `ptr` must be a valid device pointer from `rbnx_cuda_device_malloc`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_cuda_device_free(ptr: *mut u8) -> c_int {
    match unsafe { cuda::device_free(ptr) } {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

/// # Safety
/// `dst` must be a valid device pointer, `src` a valid host pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_cuda_memcpy_h2d(dst: *mut u8, src: *const u8, size: usize) -> c_int {
    match unsafe { cuda::memcpy_h2d(dst, src, size) } {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

/// # Safety
/// `dst` must be a valid host pointer, `src` a valid device pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_cuda_memcpy_d2h(dst: *mut u8, src: *const u8, size: usize) -> c_int {
    match unsafe { cuda::memcpy_d2h(dst, src, size) } {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

/// # Safety
/// Both `dst` and `src` must be valid device pointers.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_cuda_memcpy_d2d(dst: *mut u8, src: *const u8, size: usize) -> c_int {
    match unsafe { cuda::memcpy_d2d(dst, src, size) } {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rbnx_cuda_device_sync() -> c_int {
    match cuda::device_synchronize() {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

// ─── CUDA IPC (C FFI) ───────────────────────────────────────────────

/// Export a device pointer as a 64-byte IPC handle.
/// Writes the handle into `handle_out`. Returns 0 on success, -1 on error.
///
/// # Safety
/// `dev_ptr` must be from `cudaMalloc`. `handle_out` must point to 64 bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_cuda_ipc_get_handle(dev_ptr: *mut u8, handle_out: *mut u8) -> c_int {
    match unsafe { cuda::ipc_get_mem_handle(dev_ptr) } {
        Ok(handle) => {
            unsafe { std::ptr::copy_nonoverlapping(handle.as_ptr(), handle_out, 64) };
            0
        }
        Err(_) => -1,
    }
}

/// Import a device pointer from a 64-byte IPC handle.
/// Writes the device pointer into `ptr_out`. Returns 0 on success, -1 on error.
///
/// # Safety
/// `handle_in` must point to a valid 64-byte IPC handle. `ptr_out` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_cuda_ipc_open_handle(
    handle_in: *const u8,
    ptr_out: *mut *mut u8,
) -> c_int {
    let mut handle: cuda::CudaIpcMemHandle = [0u8; 64];
    unsafe { std::ptr::copy_nonoverlapping(handle_in, handle.as_mut_ptr(), 64) };
    match unsafe { cuda::ipc_open_mem_handle(handle) } {
        Ok(ptr) => {
            unsafe { *ptr_out = ptr };
            0
        }
        Err(_) => -1,
    }
}

/// Release a device pointer obtained from IPC.
///
/// # Safety
/// `dev_ptr` must have been obtained from `rbnx_cuda_ipc_open_handle`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rbnx_cuda_ipc_close_handle(dev_ptr: *mut u8) -> c_int {
    match unsafe { cuda::ipc_close_mem_handle(dev_ptr) } {
        Ok(()) => 0,
        Err(_) => -1,
    }
}
