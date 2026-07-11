import subprocess
import unittest
from unittest import mock

from audio_driver.speaker_driver import SpeakerDriver


class SpeakerDriverStopTest(unittest.TestCase):
    def test_stop_closes_stdin_and_allows_normal_drain(self):
        driver = SpeakerDriver("null")
        process = mock.Mock()
        process.wait.return_value = 0
        driver._process = process

        driver.stop()

        process.stdin.close.assert_called_once_with()
        process.wait.assert_called_once_with(timeout=5)
        process.terminate.assert_not_called()
        process.kill.assert_not_called()
        self.assertFalse(driver.is_running)

    def test_stop_terminates_then_kills_a_stuck_player(self):
        driver = SpeakerDriver("null")
        process = mock.Mock()
        process.wait.side_effect = [
            subprocess.TimeoutExpired("aplay", 5),
            subprocess.TimeoutExpired("aplay", 1),
            0,
        ]
        driver._process = process

        driver.stop()

        process.terminate.assert_called_once_with()
        process.kill.assert_called_once_with()
        self.assertEqual(process.wait.call_count, 3)
        self.assertFalse(driver.is_running)


if __name__ == "__main__":
    unittest.main()
