# SPDX-License-Identifier: MulanPSL-2.0
"""Copy into a hardware package and replace the two SDK integration points."""
from chronos_query import capture_query


class HardwareClock:
    """One hardware SDK connection; read raw ticks and leave wrap expansion to Chronos."""

    def __init__(self, options):
        """Open the device through its existing SDK using configured string options."""
        # Replace with your vendor's connection API; do not open a competing data pipeline.
        raise NotImplementedError("replace with hardware connection code")

    def read(self):
        """Bracket the real local hardware query, not a cached frame timestamp."""
        return capture_query(self.device.read_clock_ticks, device_generation=self.device_generation)

    def close(self):
        """Release the connection even if read fails."""
        self.device.close()


def create(options):
    """Configured module:factory receives only its own options and returns a read/close adapter."""
    return HardwareClock(options)
