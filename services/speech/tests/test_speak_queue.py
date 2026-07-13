# SPDX-License-Identifier: MulanPSL-2.0
import threading
import unittest

from speech_service.service import _speaker_lock


class SpeakerQueueTest(unittest.TestCase):
    def test_same_provider_serializes_callers(self):
        lock = _speaker_lock("robot-speaker")
        entered = threading.Event()

        def waiter():
            with _speaker_lock("robot-speaker"):
                entered.set()

        with lock:
            thread = threading.Thread(target=waiter)
            thread.start()
            self.assertFalse(entered.wait(0.05))
        self.assertTrue(entered.wait(0.5))
        thread.join(timeout=0.5)
        self.assertFalse(thread.is_alive())

    def test_different_providers_do_not_block_each_other(self):
        first = _speaker_lock("robot-speaker-a")
        second = _speaker_lock("robot-speaker-b")
        self.assertIsNot(first, second)
        with first:
            self.assertTrue(second.acquire(blocking=False))
            second.release()


if __name__ == "__main__":
    unittest.main()
