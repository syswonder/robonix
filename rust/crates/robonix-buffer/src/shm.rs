use libc::{
    MAP_FAILED, MAP_SHARED, O_CREAT, O_RDWR, PROT_READ, PROT_WRITE, c_int, c_void, close,
    ftruncate, mmap, munmap, shm_open, shm_unlink,
};
use std::ffi::CString;
use std::ptr;

use crate::error::{BufferError, Result};

/// A POSIX shared memory region managed by Robonix.
///
/// The region is created via `shm_open` + `ftruncate` + `mmap` and lives
/// at `/dev/shm/<name>`. Multiple processes can map the same region once
/// they know the name (provided by the control plane).
pub struct ShmRegion {
    name: CString,
    ptr: *mut u8,
    len: usize,
    owner: bool,
}

// SAFETY: The mmap'd pointer is process-shared memory. We enforce single-writer
// semantics at the BufferManager level, and reads are done through atomic
// sequencing in the header. The pointer itself is safe to send across threads.
unsafe impl Send for ShmRegion {}
unsafe impl Sync for ShmRegion {}

impl ShmRegion {
    /// Create a new shared memory region (producer side).
    pub fn create(name: &str, size: usize) -> Result<Self> {
        let cname = CString::new(name).map_err(|_| BufferError::InvalidName(name.into()))?;

        unsafe {
            let fd = shm_open(cname.as_ptr(), O_CREAT | O_RDWR, 0o666);
            if fd < 0 {
                return Err(BufferError::ShmOpen(std::io::Error::last_os_error()));
            }

            if ftruncate(fd, size as libc::off_t) < 0 {
                close(fd);
                shm_unlink(cname.as_ptr());
                return Err(BufferError::Ftruncate(std::io::Error::last_os_error()));
            }

            let ptr = mmap(
                ptr::null_mut(),
                size,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                fd,
                0,
            );
            close(fd);

            if ptr == MAP_FAILED {
                shm_unlink(cname.as_ptr());
                return Err(BufferError::Mmap(std::io::Error::last_os_error()));
            }

            Ok(Self {
                name: cname,
                ptr: ptr as *mut u8,
                len: size,
                owner: true,
            })
        }
    }

    /// Open an existing shared memory region (consumer side).
    pub fn open(name: &str, size: usize) -> Result<Self> {
        let cname = CString::new(name).map_err(|_| BufferError::InvalidName(name.into()))?;

        unsafe {
            let fd = shm_open(cname.as_ptr(), O_RDWR as c_int, 0o666);
            if fd < 0 {
                return Err(BufferError::ShmOpen(std::io::Error::last_os_error()));
            }

            let ptr = mmap(
                ptr::null_mut(),
                size,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                fd,
                0,
            );
            close(fd);

            if ptr == MAP_FAILED {
                return Err(BufferError::Mmap(std::io::Error::last_os_error()));
            }

            Ok(Self {
                name: cname,
                ptr: ptr as *mut u8,
                len: size,
                owner: false,
            })
        }
    }

    #[inline]
    pub fn as_ptr(&self) -> *mut u8 {
        self.ptr
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        unsafe { std::slice::from_raw_parts(self.ptr, self.len) }
    }

    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.len) }
    }
}

impl Drop for ShmRegion {
    fn drop(&mut self) {
        unsafe {
            if !self.ptr.is_null() {
                munmap(self.ptr as *mut c_void, self.len);
            }
            if self.owner {
                shm_unlink(self.name.as_ptr());
            }
        }
    }
}
