#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Voiceprint gRPC service entry point (atlas-codegen flavour).

Replaces the original bespoke-proto implementation (system/voiceprint/proto/
voiceprint.proto + voiceprint_pb2) with the standard robonix v0.1 flow:

  1. capabilities/system/speech/voiceprint{,_enroll,_list}.v1.toml declares
     the contract ids + IDL paths.
  2. rbnx codegen materialises generated stubs into
     rbnx-build/codegen/proto_gen/robonix_contracts{,_pb2,_pb2_grpc}.py
     (one big proto bundle, three gRPC services).
  3. This file imports those generated stubs, subclasses each Servicer
     base, and `attach_grpc_servicer`s them onto a single
     robonix_api.Service. Atlas registration + Driver(CMD_INIT) +
     heartbeat all run through Service.run() — no hand-rolled
     grpc.server() any more.

Three RPCs land on the same listener under the same atlas Service:

  * robonix/system/speech/voiceprint           — Identify(...)
      Frozen v0.1 capability surface; Liaison's voice path resolves it
      via atlas and calls it directly.
  * robonix/system/speech/voiceprint_enroll    — Enroll(...)
      Admin RPC, used by `rbnx chat`'s Ctrl+U modal to register a fresh
      sample. Mutates enrolled.json on success.
  * robonix/system/speech/voiceprint_list      — ListEnrolled(...)
      Read-only catalog; chat hydrates the user list with this on open.

State storage is unchanged: a single JSON file (`<data_dir>/enrolled.json`)
mapping `user_id -> {"name": str, "embedding": [float, ...]}`. No schema
migration needed from the previous version.

Environment variables consumed at startup:

  VOICEPRINT_DATA_DIR    default <pkg>/rbnx-build/data
  VOICEPRINT_THRESHOLD   default 0.25 (cosine similarity gate)
  VOICEPRINT_DEVICE      cuda:0 | cpu (auto-detected when unset)
"""
from __future__ import annotations

import json
import logging
import os
from pathlib import Path

import numpy as np

# Atlas-codegen stubs — produced by `rbnx build -p voiceprint` into
# rbnx-build/codegen/proto_gen/ and put on sys.path by robonix_api at
# import time. Stub class names follow PascalCase of the contract_id
# segments (see rust/crates/robonix-codegen/src/contract_gen.rs).
import robonix_contracts_pb2 as pb  # type: ignore[import-not-found]
import robonix_contracts_pb2_grpc as pb_grpc  # type: ignore[import-not-found]
from robonix_api import Service, Ok, Err  # noqa: E402

from voiceprint_service.engine import EcapaTdnnEngine

logging.basicConfig(
    level=logging.INFO,
    format="[voiceprint] %(asctime)s %(levelname)s %(message)s",
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
        log.warning(
            "Invalid VOICEPRINT_THRESHOLD=%r; falling back to %.2f",
            raw, _DEFAULT_THRESHOLD,
        )
        return _DEFAULT_THRESHOLD


# ---------------------------------------------------------------------------
# Enrolled-speaker database (JSON-backed, unchanged from PR #47)
# ---------------------------------------------------------------------------


class EnrolledDB:
    """JSON-backed speaker embedding store at `<data_dir>/enrolled.json`."""

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
        log.info(
            "Enrolled %s (%s); embedding_dim=%d",
            user_id, user_name, len(embedding),
        )

    def identify(self, query_emb: np.ndarray, threshold: float):
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
# Module-level singletons populated in on_init. None until Driver(CMD_INIT).
# ---------------------------------------------------------------------------

_engine: EcapaTdnnEngine | None = None
_db: EnrolledDB | None = None
_threshold_value: float = _DEFAULT_THRESHOLD


# ---------------------------------------------------------------------------
# gRPC servicers — one per atlas-registered contract, all sharing _engine + _db.
#
# Each Servicer subclasses a codegen-generated base class. Method names mirror
# the .srv filename (PascalCase), and the request/response types live in the
# generated pb module as `<ContractPascal>_Request` / `_Response`.
# ---------------------------------------------------------------------------


class _IdentifyServicer(pb_grpc.RobonixSystemSpeechVoiceprintServicer):
    """robonix/system/speech/voiceprint — Identify(audio) → (user_id, confidence)."""

    def Identify(self, request, context):  # noqa: N802 (gRPC method)
        if _engine is None or _db is None:
            return pb.RobonixSystemSpeechVoiceprint_Response(
                user_id="", user_name="", confidence=0.0, is_known=False,
                error="voiceprint engine not initialised (Driver(CMD_INIT) not received yet)",
            )
        try:
            emb = _engine.extract_from_pcm(
                request.audio_data,
                request.encoding or "pcm_s16le",
                request.sample_rate_hz or 16000,
            )
            uid, name, score, is_known = _db.identify(emb, _threshold_value)
            log.info(
                "Identify: best=%s (%s) score=%.4f is_known=%s",
                uid, name, score, is_known,
            )
            return pb.RobonixSystemSpeechVoiceprint_Response(
                user_id=uid, user_name=name, confidence=score,
                is_known=is_known, error="",
            )
        except Exception as exc:  # noqa: BLE001
            log.exception("Identify failed")
            return pb.RobonixSystemSpeechVoiceprint_Response(
                user_id="", user_name="", confidence=0.0,
                is_known=False, error=str(exc),
            )


class _EnrollServicer(pb_grpc.RobonixSystemSpeechVoiceprintEnrollServicer):
    """robonix/system/speech/voiceprint_enroll — Enroll(audio + user_id + user_name)."""

    def Enroll(self, request, context):  # noqa: N802
        if _engine is None or _db is None:
            return pb.RobonixSystemSpeechVoiceprintEnroll_Response(
                success=False, error="engine not initialised",
            )
        if not request.user_id or not request.audio_data:
            return pb.RobonixSystemSpeechVoiceprintEnroll_Response(
                success=False, error="user_id and audio_data are required",
            )
        try:
            emb = _engine.extract_from_pcm(
                request.audio_data,
                request.encoding or "pcm_s16le",
                request.sample_rate_hz or 16000,
            )
            _db.enroll(request.user_id, request.user_name, emb)
            return pb.RobonixSystemSpeechVoiceprintEnroll_Response(success=True, error="")
        except Exception as exc:  # noqa: BLE001
            log.exception("Enroll failed")
            return pb.RobonixSystemSpeechVoiceprintEnroll_Response(success=False, error=str(exc))


class _ListServicer(pb_grpc.RobonixSystemSpeechVoiceprintListServicer):
    """robonix/system/speech/voiceprint_list — ListEnrolled() → JSON catalog."""

    def ListEnrolled(self, request, context):  # noqa: N802
        if _db is None:
            return pb.RobonixSystemSpeechVoiceprintList_Response(
                users_json="[]", count=0, error="db not initialised",
            )
        try:
            users = [{"user_id": uid, "user_name": name} for uid, name in _db.list_users()]
            return pb.RobonixSystemSpeechVoiceprintList_Response(
                users_json=json.dumps(users, ensure_ascii=False),
                count=len(users), error="",
            )
        except Exception as exc:  # noqa: BLE001
            log.exception("ListEnrolled failed")
            return pb.RobonixSystemSpeechVoiceprintList_Response(
                users_json="[]", count=0, error=str(exc),
            )


# ---------------------------------------------------------------------------
# robonix-api Service wiring
# ---------------------------------------------------------------------------

voiceprint = Service(id="voiceprint", namespace="robonix/system/speech")

voiceprint.attach_grpc_servicer(
    "robonix/system/speech/voiceprint", _IdentifyServicer(),
)
voiceprint.attach_grpc_servicer(
    "robonix/system/speech/voiceprint_enroll", _EnrollServicer(),
)
voiceprint.attach_grpc_servicer(
    "robonix/system/speech/voiceprint_list", _ListServicer(),
)


@voiceprint.on_init
def init(cfg: dict):
    """Load the ECAPA-TDNN model + the enrolled DB. Slow (model load can
    take 10-30s the first time on Jetson Orin), but happens once at
    Driver(CMD_INIT). Subsequent Identify/Enroll calls are fast."""
    global _engine, _db, _threshold_value
    try:
        data_dir = Path(cfg.get("data_dir", str(_data_dir())))
        data_dir.mkdir(parents=True, exist_ok=True)
        _threshold_value = float(cfg.get("threshold", _threshold()))
        _engine = EcapaTdnnEngine(device=cfg.get("device"))
        _db = EnrolledDB(data_dir / "enrolled.json")
        log.info(
            "voiceprint init complete: data_dir=%s threshold=%.2f enrolled=%d",
            data_dir, _threshold_value, len(_db.data),
        )
        return Ok()
    except Exception as exc:  # noqa: BLE001
        log.exception("voiceprint init failed")
        return Err(f"voiceprint init failed: {exc}")


def main() -> int:
    voiceprint.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
