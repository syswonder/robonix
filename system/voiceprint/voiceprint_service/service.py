#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Voiceprint gRPC service entry point.

Implements the three RPCs defined in ``proto/voiceprint.proto``:

* ``Identify``       — mirrors the frozen ``robonix/system/speech/voiceprint``
  capability; consumed by Liaison's voice gating path.
* ``Enroll``         — admin RPC; extracts the speaker embedding from the
  supplied audio and persists it to the on-disk JSON database.
* ``ListEnrolled``   — admin RPC; returns the (user_id, user_name) list.

State storage is a single JSON file (``<data_dir>/enrolled.json``) of the
form ``{user_id: {"name": str, "embedding": [float, ...]}}``.

Environment variables consumed by ``main()``:

* ``VOICEPRINT_PORT``          default ``50092``
* ``VOICEPRINT_BIND_ADDR``     default ``0.0.0.0``
* ``VOICEPRINT_DATA_DIR``      default ``<pkg>/rbnx-build/data``
* ``VOICEPRINT_THRESHOLD``     default ``0.25`` (cosine similarity gate)
* ``VOICEPRINT_DEVICE``        ``cuda:0`` | ``cpu`` (auto-detected)
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import signal
from concurrent import futures
from pathlib import Path
from typing import Optional

import grpc
import numpy as np

# Importing the package runs the stub-bootstrap, so the next two imports
# always resolve regardless of whether stubs were pre-generated.
from voiceprint_service import engine as _engine_mod  # noqa: F401  (ensures pkg init)
import voiceprint_pb2
import voiceprint_pb2_grpc
from voiceprint_service.engine import EcapaTdnnEngine

logging.basicConfig(
    level=logging.INFO,
    format="[voiceprint-service] %(asctime)s %(levelname)s %(message)s",
)
log = logging.getLogger("voiceprint_service")


_PKG_DIR = Path(__file__).resolve().parent.parent
_DEFAULT_DATA_DIR = _PKG_DIR / "rbnx-build" / "data"
_DEFAULT_THRESHOLD = 0.25


def _data_dir() -> Path:
    return Path(os.environ.get("VOICEPRINT_DATA_DIR", str(_DEFAULT_DATA_DIR)))


def _threshold() -> float:
    raw = os.environ.get("VOICEPRINT_THRESHOLD")
    if raw is None:
        return _DEFAULT_THRESHOLD
    try:
        return float(raw)
    except ValueError:
        log.warning("Invalid VOICEPRINT_THRESHOLD=%r; falling back to %.2f", raw, _DEFAULT_THRESHOLD)
        return _DEFAULT_THRESHOLD


# ---------------------------------------------------------------------------
# Enrolled-speaker database
# ---------------------------------------------------------------------------


class EnrolledDB:
    """JSON-backed speaker embedding store."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self.data: dict[str, dict] = {}
        self._load()

    def _load(self) -> None:
        if self.path.is_file():
            with self.path.open("r", encoding="utf-8") as fh:
                self.data = json.load(fh)
            log.info("Loaded %d enrolled users from %s", len(self.data), self.path)
        else:
            self.data = {}

    def _save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(self.path.suffix + ".tmp")
        with tmp.open("w", encoding="utf-8") as fh:
            json.dump(self.data, fh, ensure_ascii=False, indent=2)
        os.replace(tmp, self.path)

    def enroll(self, user_id: str, user_name: str, embedding: np.ndarray) -> None:
        self.data[user_id] = {
            "name": user_name,
            "embedding": [float(x) for x in embedding.tolist()],
        }
        self._save()
        log.info("Enrolled %s (%s); embedding_dim=%d", user_id, user_name, len(embedding))

    def identify(self, query_emb: np.ndarray, threshold: float) -> tuple[str, str, float, bool]:
        best_id, best_name, best_score = "", "", -1.0
        q = np.asarray(query_emb, dtype=np.float32)
        q_norm = float(np.linalg.norm(q)) or 1.0
        for uid, entry in self.data.items():
            ref = np.asarray(entry["embedding"], dtype=np.float32)
            ref_norm = float(np.linalg.norm(ref)) or 1.0
            score = float(np.dot(q, ref) / (q_norm * ref_norm))
            if score > best_score:
                best_id, best_name, best_score = uid, entry.get("name", ""), score
        return best_id, best_name, best_score, best_score >= threshold

    def list_users(self) -> list[tuple[str, str]]:
        return [(uid, entry.get("name", "")) for uid, entry in self.data.items()]


# ---------------------------------------------------------------------------
# gRPC servicer
# ---------------------------------------------------------------------------


class VoiceprintServicer(voiceprint_pb2_grpc.VoiceprintServiceServicer):
    """gRPC façade in front of :class:`EcapaTdnnEngine` + :class:`EnrolledDB`."""

    def __init__(self, engine: EcapaTdnnEngine, db: EnrolledDB, threshold: float) -> None:
        self.engine = engine
        self.db = db
        self.threshold = threshold

    def Enroll(self, request, context):  # noqa: N802 (gRPC method name)
        try:
            embedding = self.engine.extract_from_pcm(
                request.audio_data,
                request.encoding or "pcm_s16le",
                request.sample_rate_hz or 16000,
            )
            self.db.enroll(request.user_id, request.user_name, embedding)
            return voiceprint_pb2.EnrollResponse(success=True, error="")
        except Exception as exc:
            log.exception("Enroll failed")
            return voiceprint_pb2.EnrollResponse(success=False, error=str(exc))

    def Identify(self, request, context):  # noqa: N802
        try:
            embedding = self.engine.extract_from_pcm(
                request.audio_data,
                request.encoding or "pcm_s16le",
                request.sample_rate_hz or 16000,
            )
            uid, name, score, is_known = self.db.identify(embedding, self.threshold)
            log.info(
                "Identify: best=%s (%s) score=%.4f is_known=%s",
                uid, name, score, is_known,
            )
            return voiceprint_pb2.IdentifyResponse(
                user_id=uid,
                user_name=name,
                confidence=score,
                is_known=is_known,
                error="",
            )
        except Exception as exc:
            log.exception("Identify failed")
            return voiceprint_pb2.IdentifyResponse(
                user_id="", user_name="", confidence=0.0, is_known=False, error=str(exc),
            )

    def ListEnrolled(self, request, context):  # noqa: N802
        users = [
            voiceprint_pb2.EnrolledUser(user_id=uid, user_name=name)
            for uid, name in self.db.list_users()
        ]
        return voiceprint_pb2.ListEnrolledResponse(users=users)


# ---------------------------------------------------------------------------
# Lifecycle helpers
# ---------------------------------------------------------------------------


def build_server(
    bind_addr: str,
    port: int,
    data_dir: Path,
    threshold: float,
    engine: Optional[EcapaTdnnEngine] = None,
    max_workers: int = 4,
) -> tuple[grpc.Server, EcapaTdnnEngine, EnrolledDB, int]:
    """Build (but do not start) a gRPC server with the voiceprint service.

    Returns the server, the engine, the database, and the bound port.
    A port of ``0`` triggers OS-assigned port selection; the actual port
    is returned so tests and callers can address the listener.
    """
    data_dir.mkdir(parents=True, exist_ok=True)
    enrolled_path = data_dir / "enrolled.json"

    eng = engine if engine is not None else EcapaTdnnEngine()
    db = EnrolledDB(enrolled_path)

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=max_workers))
    voiceprint_pb2_grpc.add_VoiceprintServiceServicer_to_server(
        VoiceprintServicer(eng, db, threshold), server,
    )
    bound = server.add_insecure_port(f"{bind_addr}:{port}")
    return server, eng, db, bound


def serve(
    bind_addr: str = "0.0.0.0",
    port: int = 50092,
    data_dir: Optional[Path] = None,
    threshold: Optional[float] = None,
    engine: Optional[EcapaTdnnEngine] = None,
) -> tuple[grpc.Server, EcapaTdnnEngine, EnrolledDB, int]:
    """Build, start, and return a voiceprint gRPC server."""
    data_dir = Path(data_dir) if data_dir is not None else _data_dir()
    thr = threshold if threshold is not None else _threshold()
    server, eng, db, bound = build_server(bind_addr, port, data_dir, thr, engine=engine)
    server.start()
    log.info(
        "Voiceprint gRPC server listening on %s:%d (threshold=%.2f, data_dir=%s)",
        bind_addr, bound, thr, data_dir,
    )
    return server, eng, db, bound


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Robonix voiceprint gRPC service")
    parser.add_argument("--bind", default=os.environ.get("VOICEPRINT_BIND_ADDR", "0.0.0.0"))
    parser.add_argument(
        "--port", type=int,
        default=int(os.environ.get("VOICEPRINT_PORT", "50092")),
    )
    parser.add_argument(
        "--data-dir",
        default=os.environ.get("VOICEPRINT_DATA_DIR", str(_DEFAULT_DATA_DIR)),
    )
    parser.add_argument("--threshold", type=float, default=None)
    args = parser.parse_args(argv)

    server, _, _, _ = serve(
        bind_addr=args.bind,
        port=args.port,
        data_dir=Path(args.data_dir),
        threshold=args.threshold,
    )

    stop_evt = {"requested": False}

    def _stop(signum, _frame):  # noqa: ANN001
        log.info("Received signal %s, shutting down...", signum)
        stop_evt["requested"] = True
        server.stop(grace=2).wait()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        if not stop_evt["requested"]:
            server.stop(grace=2).wait()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
