from __future__ import annotations

import sys
import types
import unittest
from unittest.mock import patch

from memsearch_service.onnx_compat import configure_onnxruntime


class _SessionOptions:
    def __init__(self) -> None:
        self.intra_op_num_threads = 0
        self.inter_op_num_threads = 0


class _InferenceSession:
    def __init__(
        self,
        path_or_bytes,
        sess_options=None,
        providers=None,
        provider_options=None,
        **kwargs,
    ) -> None:
        self.path_or_bytes = path_or_bytes
        self.sess_options = sess_options
        self.providers = providers
        self.provider_options = provider_options
        self.kwargs = kwargs


class OnnxCompatTest(unittest.TestCase):
    def test_aarch64_sessions_get_explicit_threads(self) -> None:
        ort = types.SimpleNamespace(
            InferenceSession=_InferenceSession,
            SessionOptions=_SessionOptions,
        )
        with (
            patch.dict(sys.modules, {"onnxruntime": ort}),
            patch("memsearch_service.onnx_compat.platform.machine", return_value="aarch64"),
            patch.dict("os.environ", {"MEMSEARCH_ONNX_THREADS": "2"}),
        ):
            configure_onnxruntime()
            session = ort.InferenceSession("model.onnx")
            self.assertIsInstance(session, _InferenceSession)
            self.assertEqual(session.sess_options.intra_op_num_threads, 2)
            self.assertEqual(session.sess_options.inter_op_num_threads, 1)

            wrapped = ort.InferenceSession
            configure_onnxruntime()
            self.assertIs(ort.InferenceSession, wrapped)

    def test_non_aarch64_is_unchanged(self) -> None:
        ort = types.SimpleNamespace(
            InferenceSession=_InferenceSession,
            SessionOptions=_SessionOptions,
        )
        with (
            patch.dict(sys.modules, {"onnxruntime": ort}),
            patch("memsearch_service.onnx_compat.platform.machine", return_value="x86_64"),
        ):
            configure_onnxruntime()
            self.assertIs(ort.InferenceSession, _InferenceSession)


if __name__ == "__main__":
    unittest.main()
