# SPDX-License-Identifier: MulanPSL-2.0
"""Internal hardware-query interface; adapters import this without ROS or vendor dependencies."""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable, Protocol


@dataclass(frozen=True)
class QuerySample:
    """Raw counter and actual query bracket on the Chronos host, before RPC return/queuing.

    The counter must match the data timestamp's clock domain. A confirmed hardware
    restart increases device_generation. Adapters never unwrap the counter.
    uncertainty_ns bounds extra uncertainty beyond the query bracket.
    """
    ticks: int
    before_reference_ns: int
    after_reference_ns: int
    before_steady_ns: int
    after_steady_ns: int
    device_generation: int = 1
    uncertainty_ns: int = 1


class QueryAdapter(Protocol):
    """One configured adapter owns its resources until close; failures must propagate."""

    def read(self) -> QuerySample:
        """Perform one bounded hardware query and return its real host clock bracket."""
        ...

    def close(self) -> None:
        """Release device or transport resources, including after read failure."""
        ...


def capture_query(read_ticks: Callable[[], int], device_generation: int = 1,
                  uncertainty_ns: int = 1) -> QuerySample:
    """Bracket a local query conservatively; remote driver adapters must bracket at the driver.

    Both reference reads are inside the steady bracket. Collector validation rejects
    a wall-clock discontinuity or excessive duration instead of fitting that sample.
    """
    before_steady = time.monotonic_ns()
    before_reference = time.time_ns()
    ticks = read_ticks()
    after_reference = time.time_ns()
    after_steady = time.monotonic_ns()
    return QuerySample(ticks, before_reference, after_reference, before_steady,
                       after_steady, device_generation, uncertainty_ns)
