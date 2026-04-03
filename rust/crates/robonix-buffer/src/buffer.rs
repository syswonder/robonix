use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

/// Buffer element format.
///
/// Covers image pixels, tensor elements, point cloud scalars, and
/// raw byte blobs. Values are defined by `robonix_msg/msg/BufferFormat.msg`
/// in the `crates/robonix-interfaces` IDL — keep them in sync.
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufferFormat {
    // ── Image formats (0–10) ──
    Rgb8 = 0,
    Bgr8 = 1,
    Float32 = 2,
    Nv12 = 3,
    DepthU16 = 4,
    DepthF32 = 5,
    Mono8 = 6,
    Rgba8 = 7,
    Bgra8 = 8,
    Mono16 = 9,
    F16 = 10,
    // ── Tensor / general formats (16+) ──
    BFloat16 = 16,
    Int8 = 17,
    Int32 = 18,
    Int64 = 19,
    Float64 = 20,
    /// Opaque raw bytes — consumer interprets via metadata_json.
    RawBytes = 255,
}

impl BufferFormat {
    pub fn bytes_per_element(self) -> usize {
        match self {
            Self::Rgb8
            | Self::Bgr8
            | Self::Nv12
            | Self::Mono8
            | Self::Rgba8
            | Self::Bgra8
            | Self::Int8
            | Self::RawBytes => 1,
            Self::DepthU16 | Self::Mono16 | Self::F16 | Self::BFloat16 => 2,
            Self::Float32 | Self::DepthF32 | Self::Int32 => 4,
            Self::Int64 | Self::Float64 => 8,
        }
    }

    pub fn from_u32(v: u32) -> Option<Self> {
        match v {
            0 => Some(Self::Rgb8),
            1 => Some(Self::Bgr8),
            2 => Some(Self::Float32),
            3 => Some(Self::Nv12),
            4 => Some(Self::DepthU16),
            5 => Some(Self::DepthF32),
            6 => Some(Self::Mono8),
            7 => Some(Self::Rgba8),
            8 => Some(Self::Bgra8),
            9 => Some(Self::Mono16),
            10 => Some(Self::F16),
            16 => Some(Self::BFloat16),
            17 => Some(Self::Int8),
            18 => Some(Self::Int32),
            19 => Some(Self::Int64),
            20 => Some(Self::Float64),
            255 => Some(Self::RawBytes),
            _ => None,
        }
    }

    /// ROS 2 encoding string equivalent (as used in sensor_msgs/Image).
    /// For tensor-only formats, returns a dtype-like string.
    pub fn ros_encoding(self) -> &'static str {
        match self {
            Self::Rgb8 => "rgb8",
            Self::Bgr8 => "bgr8",
            Self::Float32 => "32FC1",
            Self::Nv12 => "nv12",
            Self::DepthU16 => "16UC1",
            Self::DepthF32 => "32FC1",
            Self::Mono8 => "mono8",
            Self::Rgba8 => "rgba8",
            Self::Bgra8 => "bgra8",
            Self::Mono16 => "mono16",
            Self::F16 => "16FC1",
            Self::BFloat16 => "bf16",
            Self::Int8 => "int8",
            Self::Int32 => "int32",
            Self::Int64 => "int64",
            Self::Float64 => "float64",
            Self::RawBytes => "raw",
        }
    }
}

/// Memory domain where a buffer resides.
///
/// Values match `robonix_msg/msg/MemoryDomain.msg`.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryDomain {
    Cpu = 0,
    Gpu = 1,
    Unified = 2,
}

/// Specification for allocating a new buffer.
#[derive(Debug, Clone)]
pub struct BufferSpec {
    pub width: u32,
    pub height: u32,
    pub channels: u32,
    pub format: BufferFormat,
}

impl BufferSpec {
    pub fn frame_bytes(&self) -> usize {
        self.width as usize
            * self.height as usize
            * self.channels as usize
            * self.format.bytes_per_element()
    }

    pub fn total_bytes(&self) -> usize {
        HEADER_SIZE + self.frame_bytes()
    }

    pub fn stride(&self) -> u32 {
        self.width * self.channels * self.format.bytes_per_element() as u32
    }
}

/// Fixed 64-byte header at the beginning of every Robonix buffer.
///
/// Written atomically by the producer, read by all consumers.
/// Resides in shared memory so it is visible across processes.
pub const HEADER_SIZE: usize = 64;

#[repr(C)]
pub struct BufferHeader {
    /// Monotonically increasing frame counter. Producer increments after write.
    pub write_seq: AtomicU64,
    /// Consumer acknowledgement counter.
    pub read_ack: AtomicU64,
    /// Capture timestamp in nanoseconds since UNIX epoch.
    pub timestamp_ns: AtomicU64,
    pub width: u32,
    pub height: u32,
    pub channels: u32,
    pub stride: u32,
    pub format: u32,
    pub flags: AtomicU32,
    _reserved: [u8; 12],
}

const _: () = assert!(std::mem::size_of::<BufferHeader>() == HEADER_SIZE);

impl BufferHeader {
    /// Initialize a header in-place from a raw pointer.
    ///
    /// # Safety
    /// `ptr` must point to at least `HEADER_SIZE` bytes of writable memory.
    pub unsafe fn init_at(ptr: *mut u8, spec: &BufferSpec) {
        let hdr = unsafe { &mut *(ptr as *mut BufferHeader) };
        hdr.write_seq = AtomicU64::new(0);
        hdr.read_ack = AtomicU64::new(0);
        hdr.timestamp_ns = AtomicU64::new(0);
        hdr.width = spec.width;
        hdr.height = spec.height;
        hdr.channels = spec.channels;
        hdr.stride = spec.stride();
        hdr.format = spec.format as u32;
        hdr.flags = AtomicU32::new(0);
        hdr._reserved = [0u8; 12];
    }

    /// Read the header from a raw pointer.
    ///
    /// # Safety
    /// `ptr` must point to at least `HEADER_SIZE` bytes of readable memory.
    pub unsafe fn from_ptr<'a>(ptr: *const u8) -> &'a Self {
        unsafe { &*(ptr as *const BufferHeader) }
    }

    pub fn signal_write(&self) {
        let ts = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        self.timestamp_ns.store(ts, Ordering::Release);
        self.write_seq.fetch_add(1, Ordering::Release);
    }

    pub fn current_seq(&self) -> u64 {
        self.write_seq.load(Ordering::Acquire)
    }

    pub fn set_gpu_pinned(&self, pinned: bool) {
        if pinned {
            self.flags.fetch_or(0x1, Ordering::Release);
        } else {
            self.flags.fetch_and(!0x1, Ordering::Release);
        }
    }

    pub fn is_gpu_pinned(&self) -> bool {
        self.flags.load(Ordering::Acquire) & 0x1 != 0
    }
}

/// Lifecycle state of a managed buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufferState {
    /// Allocated by the system, not yet in use.
    Free,
    /// Producer is writing to it.
    Filling,
    /// Write complete, ready for consumers.
    Ready,
    /// One or more consumers are reading / processing.
    Processing,
}
