"""ONNX Runtime compatibility for ARM64 systems with offline CPU cores."""

from __future__ import annotations

import os
import platform


def configure_onnxruntime() -> None:
    """Disable ORT's invalid default affinity selection on aarch64.

    Some Jetson power modes expose 12 possible CPUs while only CPUs 0-7 are
    online. ORT otherwise creates a physical-core thread pool and attempts to
    bind workers to offline CPUs, which can abort the process inside cpuinfo.
    """

    if platform.machine() != "aarch64":
        return

    import onnxruntime as ort

    real_session = ort.InferenceSession
    if getattr(real_session, "_robonix_arm64_safe", False):
        return

    threads = max(1, int(os.environ.get("MEMSEARCH_ONNX_THREADS", "1")))

    class SafeInferenceSession(real_session):
        _robonix_arm64_safe = True

        def __init__(
            self,
            path_or_bytes,
            sess_options=None,
            providers=None,
            provider_options=None,
            **kwargs,
        ):
            options = sess_options or ort.SessionOptions()
            if options.intra_op_num_threads == 0:
                options.intra_op_num_threads = threads
            if options.inter_op_num_threads == 0:
                options.inter_op_num_threads = 1
            super().__init__(
                path_or_bytes,
                sess_options=options,
                providers=providers,
                provider_options=provider_options,
                **kwargs,
            )

    ort.InferenceSession = SafeInferenceSession
