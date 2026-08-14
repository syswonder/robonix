import os
import unittest
from unittest.mock import patch
from urllib.parse import parse_qs, urlsplit

from speech_service.tencent_cloud import TencentRealtimeASRBackend


class TencentRuntimeConfigTest(unittest.TestCase):
    def test_signed_url_refreshes_driver_applied_runtime_config(self):
        initial = {
            "TENCENT_ASR_APPID": "1000000001",
            "TENCENTCLOUD_SECRET_ID": "old-id",
            "TENCENTCLOUD_SECRET_KEY": "old-key",
            "TENCENT_ASR_ENGINE": "16k_zh",
        }
        updated = {
            "TENCENT_ASR_APPID": "1000000002",
            "TENCENTCLOUD_SECRET_ID": "new-id",
            "TENCENTCLOUD_SECRET_KEY": "new-key",
            "TENCENT_ASR_ENGINE": "16k_zh_en",
        }
        with patch.dict(os.environ, initial, clear=True):
            backend = TencentRealtimeASRBackend()
            os.environ.update(updated)
            url = backend._signed_url()

        parsed = urlsplit(url)
        query = parse_qs(parsed.query)
        self.assertEqual(parsed.path, "/asr/v2/1000000002")
        self.assertEqual(query["secretid"], ["new-id"])
        self.assertEqual(query["engine_model_type"], ["16k_zh_en"])
        self.assertEqual(backend.creds.appid, "1000000002")
        self.assertEqual(backend.engine, "16k_zh_en")

    def test_stream_handshake_bypasses_environment_proxy(self):
        env = {
            "TENCENT_ASR_APPID": "1000000001",
            "TENCENTCLOUD_SECRET_ID": "test-id",
            "TENCENTCLOUD_SECRET_KEY": "test-key",
            "http_proxy": "http://127.0.0.1:9981",
            "all_proxy": "socks5://127.0.0.1:9981",
        }

        class FakeSocket:
            def __init__(self):
                self.messages = iter(
                    [
                        '{"code": 0, "message": "success"}',
                        '{"code": 0, "final": 1}',
                    ]
                )

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return None

            def recv(self, timeout=None):
                return next(self.messages)

            def send(self, _message):
                return None

        with patch.dict(os.environ, env, clear=True), patch(
            "speech_service.tencent_cloud.connect", return_value=FakeSocket()
        ) as connect_mock:
            backend = TencentRealtimeASRBackend()
            list(backend.recognize_stream(iter(())))

        self.assertIsNone(connect_mock.call_args.kwargs["proxy"])


if __name__ == "__main__":
    unittest.main()
