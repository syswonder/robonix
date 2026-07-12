import unittest

from speech_service.tencent_cloud import TencentRealtimeASRBackend


class TencentStreamingTranscriptTest(unittest.TestCase):
    def test_revisable_hypotheses_emit_only_final_snapshot(self):
        last = ""
        emitted = []
        for text, final in [
            ("okay, I may", False),
            ("okay, I have seen him", False),
            ("okay, I have seen it start", True),
        ]:
            event, last = TencentRealtimeASRBackend._to_incremental_event(
                {"text": text, "is_final": final, "event_type": int(final)}, last
            )
            if event is not None:
                emitted.append(event["text"])

        self.assertEqual(emitted, ["okay, I have seen it start"])

    def test_nonfinal_duplicate_snapshot_is_suppressed(self):
        event, last = TencentRealtimeASRBackend._to_incremental_event(
            {"text": "still there", "is_final": False}, "still there"
        )
        self.assertIsNone(event)
        self.assertEqual(last, "still there")


if __name__ == "__main__":
    unittest.main()
