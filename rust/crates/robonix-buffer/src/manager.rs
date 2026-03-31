use std::collections::HashMap;
use std::sync::atomic::{AtomicU32, Ordering};

use crate::buffer::{BufferFormat, BufferHeader, BufferSpec, BufferState, HEADER_SIZE};
use crate::cuda;
use crate::error::{BufferError, Result};
use crate::shm::ShmRegion;

/// A buffer managed by the Robonix system.
///
/// Nodes never create or destroy these directly — the [`RobonixBufferManager`]
/// owns every buffer in the system and hands out [`BufferView`]s.
struct ManagedBuffer {
    handle_id: u64,
    shm_name: String,
    region: ShmRegion,
    spec: BufferSpec,
    #[allow(dead_code)]
    state: BufferState,
    gpu_pinned: bool,
    consumer_count: AtomicU32,
}

/// Read/write view into a system-managed buffer.
///
/// Returned by [`RobonixBufferManager::attach`] — the consumer receives a
/// pointer into the shared memory region without copying any data.
pub struct BufferView {
    pub handle_id: u64,
    pub header_ptr: *const u8,
    pub data_ptr: *mut u8,
    pub data_len: usize,
    pub gpu_pinned: bool,
    pub spec: BufferSpec,
}

impl BufferView {
    pub fn header(&self) -> &BufferHeader {
        unsafe { BufferHeader::from_ptr(self.header_ptr) }
    }
}

/// The central Robonix buffer coordinator.
///
/// This is the core of the zero-copy architecture: every shared memory
/// buffer in the system is **owned and managed by this struct**. Nodes
/// request buffers through the control plane, and the manager:
///
/// 1. **Allocates** the POSIX SHM region with the right size and header.
/// 2. **Registers** the buffer with the control plane (SHM name = endpoint).
/// 3. **Pins** the region for GPU DMA when a CUDA consumer attaches.
/// 4. **Tracks** reference counts and lifecycle state.
/// 5. **Releases** the region when all consumers disconnect.
pub struct RobonixBufferManager {
    buffers: HashMap<u64, ManagedBuffer>,
    next_handle: u64,
    cuda_ok: bool,
}

impl RobonixBufferManager {
    pub fn new() -> Self {
        let cuda_ok = cuda::cuda_available();
        if cuda_ok {
            log::info!("RobonixBufferManager: CUDA available, GPU pinning enabled");
        } else {
            log::info!("RobonixBufferManager: CUDA not available, CPU-only mode");
        }
        Self {
            buffers: HashMap::new(),
            next_handle: 1,
            cuda_ok,
        }
    }

    pub fn cuda_available(&self) -> bool {
        self.cuda_ok
    }

    /// Allocate a new system-managed buffer.
    ///
    /// Called when a producer node declares an interface with
    /// `transport = "shared_memory"`. The SHM name is derived from
    /// the control plane's allocated endpoint.
    pub fn allocate(&mut self, shm_name: &str, spec: BufferSpec) -> Result<u64> {
        let handle_id = self.next_handle;
        self.next_handle += 1;

        let total = spec.total_bytes();
        let region = ShmRegion::create(shm_name, total)?;

        // Initialize the buffer header in shared memory.
        unsafe {
            BufferHeader::init_at(region.as_ptr(), &spec);
        }

        log::info!(
            "allocated buffer handle={handle_id} shm={shm_name} \
             {}x{}x{} ({} bytes + {} header)",
            spec.width,
            spec.height,
            spec.channels,
            spec.frame_bytes(),
            HEADER_SIZE,
        );

        self.buffers.insert(
            handle_id,
            ManagedBuffer {
                handle_id,
                shm_name: shm_name.to_string(),
                region,
                spec,
                state: BufferState::Free,
                gpu_pinned: false,
                consumer_count: AtomicU32::new(0),
            },
        );

        Ok(handle_id)
    }

    /// Allocate a buffer with a raw byte size (for tensors, point clouds, embeddings, etc.).
    ///
    /// Unlike `allocate` which takes image-specific width/height/channels,
    /// this accepts an arbitrary `data_bytes` size. The header stores the
    /// format and total data size; shape/stride information should be
    /// communicated via `metadata_json` in the control plane.
    pub fn allocate_raw(
        &mut self,
        shm_name: &str,
        data_bytes: usize,
        format: BufferFormat,
    ) -> Result<u64> {
        let handle_id = self.next_handle;
        self.next_handle += 1;

        let total = HEADER_SIZE + data_bytes;
        let region = ShmRegion::create(shm_name, total)?;

        let spec = BufferSpec {
            width: data_bytes as u32,
            height: 1,
            channels: 1,
            format,
        };
        unsafe {
            BufferHeader::init_at(region.as_ptr(), &spec);
        }

        log::info!(
            "allocated raw buffer handle={handle_id} shm={shm_name} \
             {data_bytes} bytes, format={format:?}",
        );

        self.buffers.insert(
            handle_id,
            ManagedBuffer {
                handle_id,
                shm_name: shm_name.to_string(),
                region,
                spec,
                state: BufferState::Free,
                gpu_pinned: false,
                consumer_count: AtomicU32::new(0),
            },
        );

        Ok(handle_id)
    }

    /// Get a write view for the producer.
    pub fn producer_view(&self, handle_id: u64) -> Result<BufferView> {
        let buf = self
            .buffers
            .get(&handle_id)
            .ok_or(BufferError::NotFound(handle_id))?;
        Ok(BufferView {
            handle_id,
            header_ptr: buf.region.as_ptr(),
            data_ptr: unsafe { buf.region.as_ptr().add(HEADER_SIZE) },
            data_len: buf.spec.frame_bytes(),
            gpu_pinned: buf.gpu_pinned,
            spec: buf.spec.clone(),
        })
    }

    /// Attach a consumer to an existing buffer.
    ///
    /// Called when the control plane negotiates a `shared_memory` channel.
    /// If `pin_for_gpu` is true and CUDA is available, the system pins the
    /// buffer's pages for DMA — this is what makes the H2D transfer fast.
    pub fn attach(&mut self, handle_id: u64, pin_for_gpu: bool) -> Result<BufferView> {
        let buf = self
            .buffers
            .get_mut(&handle_id)
            .ok_or(BufferError::NotFound(handle_id))?;

        buf.consumer_count.fetch_add(1, Ordering::Relaxed);

        // Pin for GPU DMA if requested and not already pinned.
        if pin_for_gpu && self.cuda_ok && !buf.gpu_pinned {
            let data_ptr = unsafe { buf.region.as_ptr().add(HEADER_SIZE) };
            match unsafe { cuda::host_register(data_ptr, buf.spec.frame_bytes()) } {
                Ok(()) => {
                    buf.gpu_pinned = true;
                    let hdr = unsafe { BufferHeader::from_ptr(buf.region.as_ptr()) };
                    hdr.set_gpu_pinned(true);
                    log::info!(
                        "buffer handle={}: pinned {} bytes for GPU DMA",
                        handle_id,
                        buf.spec.frame_bytes()
                    );
                }
                Err(e) => {
                    log::warn!("buffer handle={handle_id}: GPU pin failed: {e}");
                }
            }
        }

        Ok(BufferView {
            handle_id,
            header_ptr: buf.region.as_ptr(),
            data_ptr: unsafe { buf.region.as_ptr().add(HEADER_SIZE) },
            data_len: buf.spec.frame_bytes(),
            gpu_pinned: buf.gpu_pinned,
            spec: buf.spec.clone(),
        })
    }

    /// Open an existing shared memory buffer created by another process.
    ///
    /// Maps the SHM region, reads the `BufferHeader` to discover geometry,
    /// and registers the buffer locally. Returns a handle that can be used
    /// with `attach`, `producer_view`, etc.  The consumer does **not** own
    /// the SHM segment (it will not be unlinked on drop).
    pub fn open(&mut self, shm_name: &str) -> Result<u64> {
        if let Some(id) = self.handle_by_name(shm_name) {
            return Ok(id);
        }

        // Phase 1: map just the header to read geometry.
        let hdr_region = ShmRegion::open(shm_name, HEADER_SIZE)?;
        let spec = unsafe {
            let hdr = BufferHeader::from_ptr(hdr_region.as_ptr());
            let fmt = BufferFormat::from_u32(hdr.format)
                .ok_or_else(|| BufferError::InvalidName(format!("bad format {}", hdr.format)))?;
            BufferSpec {
                width: hdr.width,
                height: hdr.height,
                channels: hdr.channels,
                format: fmt,
            }
        };
        drop(hdr_region);

        // Phase 2: map the full region (header + pixel data).
        let total = spec.total_bytes();
        let region = ShmRegion::open(shm_name, total)?;

        let handle_id = self.next_handle;
        self.next_handle += 1;

        log::info!(
            "opened buffer handle={handle_id} shm={shm_name} \
             {}x{}x{} ({} bytes, consumer)",
            spec.width,
            spec.height,
            spec.channels,
            spec.frame_bytes(),
        );

        self.buffers.insert(
            handle_id,
            ManagedBuffer {
                handle_id,
                shm_name: shm_name.to_string(),
                region,
                spec,
                state: BufferState::Ready,
                gpu_pinned: false,
                consumer_count: AtomicU32::new(0),
            },
        );

        Ok(handle_id)
    }

    /// Look up a buffer handle by its SHM name.
    pub fn handle_by_name(&self, shm_name: &str) -> Option<u64> {
        self.buffers
            .values()
            .find(|b| b.shm_name == shm_name)
            .map(|b| b.handle_id)
    }

    /// Detach a consumer. If the last consumer disconnects and the buffer
    /// was GPU-pinned, unpin it.
    pub fn detach(&mut self, handle_id: u64) -> Result<()> {
        let buf = self
            .buffers
            .get_mut(&handle_id)
            .ok_or(BufferError::NotFound(handle_id))?;

        let prev = buf.consumer_count.fetch_sub(1, Ordering::Relaxed);
        if prev <= 1 && buf.gpu_pinned {
            let data_ptr = unsafe { buf.region.as_ptr().add(HEADER_SIZE) };
            let _ = unsafe { cuda::host_unregister(data_ptr) };
            buf.gpu_pinned = false;
            let hdr = unsafe { BufferHeader::from_ptr(buf.region.as_ptr()) };
            hdr.set_gpu_pinned(false);
            log::info!("buffer handle={handle_id}: unpinned (last consumer detached)");
        }

        Ok(())
    }

    /// Release a buffer entirely. Unmaps SHM and unlinks the name.
    pub fn release(&mut self, handle_id: u64) -> Result<()> {
        if let Some(buf) = self.buffers.remove(&handle_id) {
            if buf.gpu_pinned {
                let data_ptr = unsafe { buf.region.as_ptr().add(HEADER_SIZE) };
                let _ = unsafe { cuda::host_unregister(data_ptr) };
            }
            log::info!("buffer handle={handle_id} shm={} released", buf.shm_name);
            // ShmRegion::drop will munmap + shm_unlink (if owner)
        }
        Ok(())
    }

    pub fn buffer_count(&self) -> usize {
        self.buffers.len()
    }

    pub fn list_buffers(&self) -> Vec<(u64, String, bool)> {
        self.buffers
            .values()
            .map(|b| (b.handle_id, b.shm_name.clone(), b.gpu_pinned))
            .collect()
    }
}

impl Default for RobonixBufferManager {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for RobonixBufferManager {
    fn drop(&mut self) {
        let ids: Vec<u64> = self.buffers.keys().copied().collect();
        for id in ids {
            let _ = self.release(id);
        }
    }
}
