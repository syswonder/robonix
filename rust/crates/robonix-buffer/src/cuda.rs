use libloading::{Library, Symbol};
use std::sync::OnceLock;

use crate::error::{BufferError, Result};

const CUDART_NAMES: &[&str] = &["libcudart.so", "libcudart.so.12", "libcudart.so.11"];

static CUDA_RT: OnceLock<Option<CudaRuntime>> = OnceLock::new();

type CudaError = i32;
const CUDA_SUCCESS: CudaError = 0;

/// 64-byte opaque handle for CUDA IPC memory sharing across processes.
pub type CudaIpcMemHandle = [u8; 64];

/// Thin wrapper around dynamically-loaded CUDA runtime functions.
///
/// Loaded lazily on first use. If `libcudart.so` is absent the system
/// gracefully degrades to CPU-only operation.
struct CudaRuntime {
    _lib: Library,

    get_device_count: unsafe extern "C" fn(*mut i32) -> CudaError,
    host_register: unsafe extern "C" fn(*mut u8, usize, u32) -> CudaError,
    host_unregister: unsafe extern "C" fn(*mut u8) -> CudaError,
    device_synchronize: unsafe extern "C" fn() -> CudaError,
    malloc: unsafe extern "C" fn(*mut *mut u8, usize) -> CudaError,
    free: unsafe extern "C" fn(*mut u8) -> CudaError,
    memcpy: unsafe extern "C" fn(*mut u8, *const u8, usize, i32) -> CudaError,
    ipc_get_mem_handle: unsafe extern "C" fn(*mut CudaIpcMemHandle, *mut u8) -> CudaError,
    ipc_open_mem_handle: unsafe extern "C" fn(*mut *mut u8, CudaIpcMemHandle, u32) -> CudaError,
    ipc_close_mem_handle: unsafe extern "C" fn(*mut u8) -> CudaError,
}

// SAFETY: CUDA runtime calls are thread-safe per NVIDIA documentation.
unsafe impl Send for CudaRuntime {}
unsafe impl Sync for CudaRuntime {}

impl CudaRuntime {
    fn load() -> Option<Self> {
        for name in CUDART_NAMES {
            if let Ok(lib) = unsafe { Library::new(name) } {
                unsafe {
                    let get_device_count: Symbol<unsafe extern "C" fn(*mut i32) -> CudaError> =
                        lib.get(b"cudaGetDeviceCount").ok()?;
                    let host_register: Symbol<
                        unsafe extern "C" fn(*mut u8, usize, u32) -> CudaError,
                    > = lib.get(b"cudaHostRegister").ok()?;
                    let host_unregister: Symbol<unsafe extern "C" fn(*mut u8) -> CudaError> =
                        lib.get(b"cudaHostUnregister").ok()?;
                    let device_synchronize: Symbol<unsafe extern "C" fn() -> CudaError> =
                        lib.get(b"cudaDeviceSynchronize").ok()?;
                    let malloc: Symbol<unsafe extern "C" fn(*mut *mut u8, usize) -> CudaError> =
                        lib.get(b"cudaMalloc").ok()?;
                    let free: Symbol<unsafe extern "C" fn(*mut u8) -> CudaError> =
                        lib.get(b"cudaFree").ok()?;
                    let memcpy: Symbol<
                        unsafe extern "C" fn(*mut u8, *const u8, usize, i32) -> CudaError,
                    > = lib.get(b"cudaMemcpy").ok()?;
                    let ipc_get_mem_handle: Symbol<
                        unsafe extern "C" fn(*mut CudaIpcMemHandle, *mut u8) -> CudaError,
                    > = lib.get(b"cudaIpcGetMemHandle").ok()?;
                    let ipc_open_mem_handle: Symbol<
                        unsafe extern "C" fn(*mut *mut u8, CudaIpcMemHandle, u32) -> CudaError,
                    > = lib.get(b"cudaIpcOpenMemHandle").ok()?;
                    let ipc_close_mem_handle: Symbol<unsafe extern "C" fn(*mut u8) -> CudaError> =
                        lib.get(b"cudaIpcCloseMemHandle").ok()?;

                    return Some(Self {
                        get_device_count: *get_device_count,
                        host_register: *host_register,
                        host_unregister: *host_unregister,
                        device_synchronize: *device_synchronize,
                        malloc: *malloc,
                        free: *free,
                        memcpy: *memcpy,
                        ipc_get_mem_handle: *ipc_get_mem_handle,
                        ipc_open_mem_handle: *ipc_open_mem_handle,
                        ipc_close_mem_handle: *ipc_close_mem_handle,
                        _lib: lib,
                    });
                }
            }
        }
        None
    }
}

fn rt() -> Result<&'static CudaRuntime> {
    CUDA_RT
        .get_or_init(CudaRuntime::load)
        .as_ref()
        .ok_or(BufferError::CudaUnavailable)
}

/// Returns `true` if a CUDA-capable GPU is present.
pub fn cuda_available() -> bool {
    rt().ok().is_some_and(|r| {
        let mut count: i32 = 0;
        unsafe { (r.get_device_count)(&mut count) == CUDA_SUCCESS && count > 0 }
    })
}

/// Pin host memory pages so the GPU DMA engine can transfer directly,
/// bypassing the CUDA driver's internal staging buffer.
///
/// # Safety
/// `ptr` must point to a valid, mapped memory region of at least `size` bytes.
pub unsafe fn host_register(ptr: *mut u8, size: usize) -> Result<()> {
    let r = rt()?;
    let err = unsafe { (r.host_register)(ptr, size, 0) };
    if err != CUDA_SUCCESS {
        return Err(BufferError::CudaCall("cudaHostRegister", err));
    }
    log::debug!("cudaHostRegister: pinned {size} bytes at {ptr:?}");
    Ok(())
}

/// Unpin previously registered host memory.
///
/// # Safety
/// `ptr` must have been previously registered with [`host_register`].
pub unsafe fn host_unregister(ptr: *mut u8) -> Result<()> {
    let r = rt()?;
    let err = unsafe { (r.host_unregister)(ptr) };
    if err != CUDA_SUCCESS {
        return Err(BufferError::CudaCall("cudaHostUnregister", err));
    }
    Ok(())
}

/// Synchronize the current CUDA device (wait for all pending GPU work).
pub fn device_synchronize() -> Result<()> {
    let r = rt()?;
    let err = unsafe { (r.device_synchronize)() };
    if err != CUDA_SUCCESS {
        return Err(BufferError::CudaCall("cudaDeviceSynchronize", err));
    }
    Ok(())
}

/// Allocate device memory.
pub fn device_malloc(size: usize) -> Result<*mut u8> {
    let r = rt()?;
    let mut ptr: *mut u8 = std::ptr::null_mut();
    let err = unsafe { (r.malloc)(&mut ptr, size) };
    if err != CUDA_SUCCESS {
        return Err(BufferError::CudaCall("cudaMalloc", err));
    }
    Ok(ptr)
}

/// Free device memory.
///
/// # Safety
/// `ptr` must be a valid device pointer from [`device_malloc`].
pub unsafe fn device_free(ptr: *mut u8) -> Result<()> {
    let r = rt()?;
    let err = unsafe { (r.free)(ptr) };
    if err != CUDA_SUCCESS {
        return Err(BufferError::CudaCall("cudaFree", err));
    }
    Ok(())
}

/// Copy from host to device.
///
/// # Safety
/// `dst` must be a valid device pointer, `src` a valid host pointer,
/// both with at least `size` bytes accessible.
pub unsafe fn memcpy_h2d(dst: *mut u8, src: *const u8, size: usize) -> Result<()> {
    let r = rt()?;
    let err = unsafe { (r.memcpy)(dst, src, size, 1) };
    if err != CUDA_SUCCESS {
        return Err(BufferError::CudaCall("cudaMemcpy(H2D)", err));
    }
    Ok(())
}

/// Copy from device to host.
///
/// # Safety
/// `dst` must be a valid host pointer, `src` a valid device pointer,
/// both with at least `size` bytes accessible.
pub unsafe fn memcpy_d2h(dst: *mut u8, src: *const u8, size: usize) -> Result<()> {
    let r = rt()?;
    let err = unsafe { (r.memcpy)(dst, src, size, 2) };
    if err != CUDA_SUCCESS {
        return Err(BufferError::CudaCall("cudaMemcpy(D2H)", err));
    }
    Ok(())
}

/// Copy within device memory (D2D, no CPU or PCIe involved).
///
/// # Safety
/// Both `dst` and `src` must be valid device pointers with at least `size` bytes.
pub unsafe fn memcpy_d2d(dst: *mut u8, src: *const u8, size: usize) -> Result<()> {
    let r = rt()?;
    let err = unsafe { (r.memcpy)(dst, src, size, 3) }; // cudaMemcpyDeviceToDevice = 3
    if err != CUDA_SUCCESS {
        return Err(BufferError::CudaCall("cudaMemcpy(D2D)", err));
    }
    Ok(())
}

// ─── CUDA IPC ────────────────────────────────────────────────────────

/// Export a device pointer as an IPC handle that another process can import.
///
/// The returned 64-byte handle can be serialized (e.g. via SHM) and passed
/// to [`ipc_open_mem_handle`] in another process on the same machine.
///
/// # Safety
/// `dev_ptr` must be a device pointer allocated with `cudaMalloc`.
pub unsafe fn ipc_get_mem_handle(dev_ptr: *mut u8) -> Result<CudaIpcMemHandle> {
    let r = rt()?;
    let mut handle: CudaIpcMemHandle = [0u8; 64];
    let err = unsafe { (r.ipc_get_mem_handle)(&mut handle, dev_ptr) };
    if err != CUDA_SUCCESS {
        return Err(BufferError::CudaCall("cudaIpcGetMemHandle", err));
    }
    Ok(handle)
}

/// Import a device pointer from an IPC handle exported by another process.
///
/// Returns a device pointer valid in the calling process's CUDA context.
/// Must be released with [`ipc_close_mem_handle`] (NOT `cudaFree`).
///
/// # Safety
/// `handle` must be a valid IPC handle from [`ipc_get_mem_handle`].
pub unsafe fn ipc_open_mem_handle(handle: CudaIpcMemHandle) -> Result<*mut u8> {
    let r = rt()?;
    let mut ptr: *mut u8 = std::ptr::null_mut();
    // cudaIpcMemLazyEnablePeerAccess = 0x01
    let err = unsafe { (r.ipc_open_mem_handle)(&mut ptr, handle, 0x01) };
    if err != CUDA_SUCCESS {
        return Err(BufferError::CudaCall("cudaIpcOpenMemHandle", err));
    }
    Ok(ptr)
}

/// Release a device pointer obtained via [`ipc_open_mem_handle`].
///
/// # Safety
/// `dev_ptr` must have been obtained from `ipc_open_mem_handle`.
pub unsafe fn ipc_close_mem_handle(dev_ptr: *mut u8) -> Result<()> {
    let r = rt()?;
    let err = unsafe { (r.ipc_close_mem_handle)(dev_ptr) };
    if err != CUDA_SUCCESS {
        return Err(BufferError::CudaCall("cudaIpcCloseMemHandle", err));
    }
    Ok(())
}
