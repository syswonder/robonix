# SPDX-License-Identifier: MulanPSL-2.0
"""Example standalone RealSense clock-query adapter; never start a second image pipeline."""
from chronos_query import capture_query


class RealSenseClock:
    """Keep one SDK context/device for low-frequency hardware-monitor queries."""

    def __init__(self, options):
        """Load the SDK only on selection and require a serial to avoid querying the wrong clock."""
        import pyrealsense2 as rs
        serial = options.get("serial")
        if not serial:
            raise ValueError("RealSense query requires an explicit serial")
        self.context = rs.context()
        devices = [device for device in self.context.query_devices()
                   if device.get_info(rs.camera_info.serial_number) == serial]
        if len(devices) != 1:
            raise RuntimeError("configured RealSense device was not uniquely found")
        self.device = devices[0]
        if not callable(getattr(self.device, "get_device_time_ms", None)):
            raise RuntimeError("installed RealSense SDK lacks get_device_time_ms()")

    def read(self):
        """Return u32 microsecond ticks; SDK float milliseconds are rounded to the hardware tick."""
        return capture_query(lambda: round(self.device.get_device_time_ms() * 1000))

    def close(self):
        """Release SDK references; no image streaming resources were started by this adapter."""
        self.device = None
        self.context = None


def create(options):
    """Factory selected as realsense:create on the collector's Python module path."""
    return RealSenseClock(options)
