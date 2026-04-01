"""Demo-specific iceoryx2 payload types for camera frame transport.

These ctypes.Structure subclasses are used as iceoryx2 publish-subscribe
payload types in the ManiSkill VLA demo.  The type_name() staticmethod is
required by the iceoryx2 Python API for type-safe service matching.

Max buffer sizes are chosen to cover the demo's common resolutions:
  - 640×480 RGB  = 921 600 bytes (~900 KB)
  - 640×480 Depth (float32) = 1 228 800 bytes (~1.2 MB)
Actual pixel data up to (width * height * channels) bytes is valid;
the trailing bytes of the fixed buffer are unused padding.
"""

import ctypes

# ── Buffer capacity constants ─────────────────────────────────────────────────

_MAX_W = 640
_MAX_H = 480
_MAX_RGB_BYTES   = _MAX_W * _MAX_H * 3        # uint8, 3 channels
_MAX_DEPTH_BYTES = _MAX_W * _MAX_H * 4        # float32, 1 channel


# ── Payload types ─────────────────────────────────────────────────────────────

class RgbCameraFrame(ctypes.Structure):
    """RGB camera frame published on service 'robonix/camera/rgb'.

    Fields
    ------
    width, height, channels : frame geometry (actual valid region of `pixels`)
    seq                     : monotonically increasing sequence number
    ts_ns                   : hardware timestamp in nanoseconds since UNIX epoch
    pixels                  : packed row-major uint8 RGB data; only the first
                              width*height*channels bytes are valid
    """

    _fields_ = [
        ("width",    ctypes.c_uint32),
        ("height",   ctypes.c_uint32),
        ("channels", ctypes.c_uint32),
        ("seq",      ctypes.c_uint64),
        ("ts_ns",    ctypes.c_uint64),
        ("pixels",   ctypes.c_uint8 * _MAX_RGB_BYTES),
    ]

    @staticmethod
    def type_name() -> str:
        return "robonix.demo.RgbCameraFrame"


class DepthCameraFrame(ctypes.Structure):
    """Depth camera frame published on service 'robonix/camera/depth'.

    Fields
    ------
    width, height : frame geometry (actual valid region of `pixels`)
    seq           : sequence number aligned with the corresponding RGB frame
    ts_ns         : hardware timestamp
    pixels        : packed row-major float32 depth data stored as raw bytes;
                    reinterpret as numpy float32 after slicing to width*height*4
    """

    _fields_ = [
        ("width",  ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("seq",    ctypes.c_uint64),
        ("ts_ns",  ctypes.c_uint64),
        ("pixels", ctypes.c_uint8 * _MAX_DEPTH_BYTES),
    ]

    @staticmethod
    def type_name() -> str:
        return "robonix.demo.DepthCameraFrame"


# ── Service names ─────────────────────────────────────────────────────────────

IOX2_SERVICE_RGB   = "robonix/camera/rgb"
IOX2_SERVICE_DEPTH = "robonix/camera/depth"
