use thiserror::Error;

pub type Result<T> = std::result::Result<T, BufferError>;

#[derive(Error, Debug)]
pub enum BufferError {
    #[error("shm_open failed: {0}")]
    ShmOpen(std::io::Error),

    #[error("ftruncate failed: {0}")]
    Ftruncate(std::io::Error),

    #[error("mmap failed: {0}")]
    Mmap(std::io::Error),

    #[error("invalid SHM name: {0}")]
    InvalidName(String),

    #[error("buffer not found: handle_id={0}")]
    NotFound(u64),

    #[error("buffer already exists: handle_id={0}")]
    AlreadyExists(u64),

    #[error("CUDA runtime not available (libcudart.so not found)")]
    CudaUnavailable,

    #[error("CUDA {0} failed with error code {1}")]
    CudaCall(&'static str, i32),
}
