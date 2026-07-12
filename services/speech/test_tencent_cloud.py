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


if __name__ == "__main__":
    unittest.main()
