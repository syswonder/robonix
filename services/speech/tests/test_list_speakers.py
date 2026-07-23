# SPDX-License-Identifier: MulanPSL-2.0
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from speech_service.service import ATLAS, Transport, list_speakers


class ListSpeakersTest(unittest.TestCase):
    def test_queries_only_grpc_speakers_with_requested_namespace_prefix(self):
        request = SimpleNamespace(namespace_prefix="robot/audio")

        with patch.object(ATLAS, "find_capability", return_value=[]) as find_capability:
            list_speakers(request)

        find_capability.assert_called_once_with(
            contract_id="robonix/primitive/audio/speaker",
            namespace_prefix="robot/audio",
            transport=Transport.GRPC,
        )


if __name__ == "__main__":
    unittest.main()
