# SPDX-License-Identifier: MulanPSL-2.0
"""End-to-end voiceprint pipeline test (Enroll → Identify → gating).

Mirrors the original out-of-tree TUI test (`test_tui.py`) without the
ASR / Pilot dependencies: it stands up an in-process gRPC voiceprint
server, enrolls two speakers (`liukaile`, `com`) from their first
samples, and asserts that

  * each speaker's second sample is identified back to them with
    ``is_known == True`` and ``confidence`` above threshold,
  * an un-enrolled speaker (``change.wav``) is rejected with
    ``is_known == False``,
  * ``ListEnrolled`` returns the two enrolled users.

The test instantiates a real ``EcapaTdnnEngine`` (SpeechBrain
ECAPA-TDNN). The model is fetched on first run; run
``bash scripts/build.sh`` first to pre-cache it offline.

Skip the suite by setting ``VOICEPRINT_SKIP_E2E=1``; useful in
CI lanes that do not have speechbrain / torch available.
"""
from __future__ import annotations

import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import soundfile as sf

_THIS = Path(__file__).resolve()
_PKG_ROOT = _THIS.parent.parent
sys.path.insert(0, str(_PKG_ROOT))

FIXTURES = _THIS.parent / "fixtures"
THRESHOLD = 0.35


def _read_pcm(path: Path) -> tuple[bytes, int]:
    audio, sr = sf.read(str(path), dtype="int16")
    return audio.tobytes(), int(sr)


@unittest.skipIf(
    os.environ.get("VOICEPRINT_SKIP_E2E") == "1",
    "VOICEPRINT_SKIP_E2E=1 set; skipping ECAPA-TDNN end-to-end test",
)
class VoiceprintPipelineTest(unittest.TestCase):
    """Spin up the gRPC service in-process and exercise all three RPCs."""

    @classmethod
    def setUpClass(cls) -> None:
        try:
            import grpc  # noqa: F401
            import voiceprint_service  # noqa: F401  (bootstraps gRPC stubs)
            import voiceprint_pb2
            import voiceprint_pb2_grpc
            from voiceprint_service.service import build_server
            from voiceprint_service.engine import EcapaTdnnEngine
        except Exception as exc:  # pragma: no cover - import-time failure
            raise unittest.SkipTest(f"voiceprint dependencies unavailable: {exc}")

        cls._grpc = __import__("grpc")
        cls._pb2 = voiceprint_pb2
        cls._pb2_grpc = voiceprint_pb2_grpc

        cls.tmp_dir = Path(tempfile.mkdtemp(prefix="voiceprint-e2e-"))
        cls.engine = EcapaTdnnEngine()
        cls.server, _, cls.db, port = build_server(
            bind_addr="127.0.0.1",
            port=0,
            data_dir=cls.tmp_dir,
            threshold=THRESHOLD,
            engine=cls.engine,
        )
        cls.server.start()
        cls.port = port
        cls.channel = cls._grpc.insecure_channel(f"127.0.0.1:{port}")
        cls.stub = cls._pb2_grpc.VoiceprintServiceStub(cls.channel)

    @classmethod
    def tearDownClass(cls) -> None:
        try:
            cls.channel.close()
            cls.server.stop(grace=0).wait()
        finally:
            shutil.rmtree(cls.tmp_dir, ignore_errors=True)

    # -- helpers -----------------------------------------------------------

    def _enroll(self, user_id: str, user_name: str, wav: Path) -> None:
        pcm, sr = _read_pcm(wav)
        resp = self.stub.Enroll(self._pb2.EnrollRequest(
            user_id=user_id, user_name=user_name,
            audio_data=pcm, encoding="pcm_s16le", sample_rate_hz=sr,
        ))
        self.assertTrue(resp.success, msg=f"Enroll({user_id}) failed: {resp.error!r}")

    def _identify(self, wav: Path):
        pcm, sr = _read_pcm(wav)
        return self.stub.Identify(self._pb2.IdentifyRequest(
            audio_data=pcm, encoding="pcm_s16le", sample_rate_hz=sr,
        ))

    # -- tests -------------------------------------------------------------

    def test_full_pipeline(self) -> None:
        """Enroll two speakers, then assert acceptance + rejection behaviour."""
        self._enroll("liukaile", "刘凯乐", FIXTURES / "liukaile.wav")
        self._enroll("com", "Com", FIXTURES / "com.wav")

        listing = self.stub.ListEnrolled(self._pb2.ListEnrolledRequest())
        self.assertEqual(
            sorted(u.user_id for u in listing.users),
            ["com", "liukaile"],
            msg="ListEnrolled should return exactly the two users we just enrolled",
        )

        # Test A: liukaile's second sample → recognised as liukaile.
        resp_a = self._identify(FIXTURES / "liukaile2.wav")
        self.assertEqual(resp_a.error, "")
        self.assertTrue(
            resp_a.is_known,
            msg=(f"liukaile2.wav should be accepted, got "
                 f"user_id={resp_a.user_id!r} confidence={resp_a.confidence:.4f}"),
        )
        self.assertEqual(resp_a.user_id, "liukaile")
        self.assertGreaterEqual(resp_a.confidence, THRESHOLD)

        # Test B: com's second sample → recognised as com.
        resp_b = self._identify(FIXTURES / "com2.wav")
        self.assertEqual(resp_b.error, "")
        self.assertTrue(resp_b.is_known, msg=f"com2.wav rejected: confidence={resp_b.confidence:.4f}")
        self.assertEqual(resp_b.user_id, "com")
        self.assertGreaterEqual(resp_b.confidence, THRESHOLD)

        # Test C: change.wav is NOT enrolled → must be rejected.
        resp_c = self._identify(FIXTURES / "change.wav")
        self.assertEqual(resp_c.error, "")
        self.assertFalse(
            resp_c.is_known,
            msg=(f"change.wav should be rejected (threshold={THRESHOLD}), got "
                 f"is_known={resp_c.is_known} confidence={resp_c.confidence:.4f}"),
        )
        self.assertLess(resp_c.confidence, THRESHOLD)


if __name__ == "__main__":
    unittest.main(verbosity=2)
