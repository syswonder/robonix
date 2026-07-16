#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Voiceprint gRPC service entry point.

The package follows the standard Robonix capability and codegen flow:

  1. ``capabilities/service/voiceprint/*.v1.toml`` declares the lifecycle and
     Voiceprint contract IDs; ``capabilities/lib/voiceprint/srv/*.srv`` defines
     their request and response messages.
  2. ``rbnx codegen`` materialises generated stubs into
     rbnx-build/codegen/proto_gen/robonix_contracts{,_pb2,_pb2_grpc}.py
     and the per-IDL Voiceprint modules.
  3. This file imports those generated stubs, subclasses each Servicer
     base, and `attach_grpc_servicer`s them onto a single
     ``robonix_api.Service``. Atlas registration, Driver(CMD_INIT), and
     heartbeat all run through ``Service.run()``.

Four Voiceprint RPCs land on the same listener under one Atlas Service:

  * robonix/service/voiceprint/identify           — Identify(...)
      Frozen v0.1 capability surface; Liaison's voice path resolves it
      via atlas and calls it directly.
  * robonix/service/voiceprint/enroll    — Enroll(...)
      Admin RPC, used by `rbnx chat`'s Ctrl+U modal to register a fresh
      sample. Mutates enrolled.json on success.
  * robonix/service/voiceprint/list      — ListEnrolled(...)
      Read-only catalog; chat hydrates the user list with this on open.
  * robonix/service/voiceprint/delete    — DeleteEnrolled(...)
      Idempotently removes one enrolled speaker.

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
import math
import os
import threading
from pathlib import Path

import numpy as np

# Atlas-codegen stubs — produced by `rbnx build -p voiceprint` into
# rbnx-build/codegen/proto_gen/ and put on sys.path by robonix_api at
# import time. Stub class names follow PascalCase of the contract_id
# segments (see tools/codegen/src/contract_gen.rs).
try:
    import robonix_contracts_pb2 as pb  # type: ignore[import-not-found]
    import robonix_contracts_pb2_grpc as pb_grpc  # type: ignore[import-not-found]
    # Voiceprint contract messages live under the per-IDL `robonix.voiceprint`
    # proto package (Identify_*, Enroll_*, ListEnrolled_*). They are NOT
    # remangled into `robonix_contracts_pb2.RobonixServiceSpeech*` — that
    # only mangles SERVICE names. Use the voiceprint_pb2 namespace for the
    # request/response dataclasses.
    import voiceprint_pb2 as vp  # type: ignore[import-not-found]
    from voiceprint_mcp import (  # type: ignore[import-not-found]
        ListEnrolled_Request,
        ListEnrolled_Response,
    )
except Exception as exc:  # noqa: BLE001 - generated modules raise version-specific exceptions
    raise RuntimeError(
        "Voiceprint generated modules are missing or incompatible; rebuild "
        "this package or its containing deployment before starting it."
    ) from exc
from robonix_api import Service, Ok, Err, scribe_logger  # noqa: E402

from voiceprint_service.engine import EcapaTdnnEngine

# Route all stdlib logging through Scribe so `rbnx logs -t voiceprint` sees the
# full trace and the package owns no log file or stdout sink of its own.
scribe_logger.install_stdlib_bridge("voiceprint")
log = logging.getLogger("voiceprint_service")


_PKG_DIR = Path(__file__).resolve().parent.parent
_DEFAULT_DATA_DIR = _PKG_DIR / "rbnx-build" / "data"
_DEFAULT_THRESHOLD = 0.25


def _data_dir() -> Path:
    return Path(os.environ.get("VOICEPRINT_DATA_DIR", str(_DEFAULT_DATA_DIR)))


def _validate_threshold(raw: object) -> float:
    """Return a finite cosine threshold in the supported [0, 1] range."""
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError("threshold must be a finite number in [0, 1]") from exc
    if not math.isfinite(value) or not 0.0 <= value <= 1.0:
        raise ValueError("threshold must be a finite number in [0, 1]")
    return value


def _threshold() -> float:
    return _validate_threshold(
        os.environ.get("VOICEPRINT_THRESHOLD", _DEFAULT_THRESHOLD),
    )


# ---------------------------------------------------------------------------
# Enrolled-speaker database (JSON-backed)
# ---------------------------------------------------------------------------


class EnrolledDB:
    """JSON-backed speaker embedding store at `<data_dir>/enrolled.json`."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self.data: dict[str, dict] = {}
        self._lock = threading.RLock()
        self._load()

    def _load(self) -> None:
        with self._lock:
            if self.path.is_file():
                with self.path.open("r", encoding="utf-8") as fh:
                    self.data = json.load(fh)
                log.info("Loaded %d enrolled users from %s", len(self.data), self.path)
            else:
                self.data = {}

    def _save_locked(self) -> None:
        """Persist the current snapshot; caller must hold ``self._lock``."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(self.path.suffix + ".tmp")
        with tmp.open("w", encoding="utf-8") as fh:
            json.dump(self.data, fh, ensure_ascii=False, indent=2)
        os.replace(tmp, self.path)

    def _identify_locked(self, query_emb: np.ndarray, threshold: float):
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

    def enroll(self, user_id: str, user_name: str, embedding: np.ndarray) -> None:
        """Persist an unconditional enrollment, rolling back on write failure."""
        with self._lock:
            missing = object()
            previous = self.data.get(user_id, missing)
            self.data[user_id] = {
                "name": user_name,
                "embedding": [float(x) for x in embedding.tolist()],
            }
            try:
                self._save_locked()
            except Exception:
                if previous is missing:
                    del self.data[user_id]
                else:
                    self.data[user_id] = previous
                raise
            log.info(
                "Enrolled %s (%s); embedding_dim=%d",
                user_id, user_name, len(embedding),
            )

    def enroll_unique(
        self,
        user_id: str,
        user_name: str,
        embedding: np.ndarray,
        threshold: float,
    ) -> tuple[bool, str]:
        """Atomically reject duplicates, mutate the snapshot, and persist it."""
        with self._lock:
            if user_id in self.data:
                existing = self.data[user_id].get("name", "")
                return (
                    False,
                    f"user_id '{user_id}' already enrolled as '{existing}'; "
                    "delete it first if you want to re-enrol",
                )

            if user_name:
                duplicate_id = next(
                    (
                        uid
                        for uid, entry in self.data.items()
                        if entry.get("name", "") == user_name
                    ),
                    None,
                )
                if duplicate_id is not None:
                    return (
                        False,
                        f"name '{user_name}' already enrolled as user_id "
                        f"'{duplicate_id}'; delete it first if you want to re-enrol",
                    )

            if self.data:
                best_id, best_name, best_score, is_match = self._identify_locked(
                    embedding,
                    threshold,
                )
                if is_match:
                    return (
                        False,
                        f"voice already enrolled as '{best_name}' "
                        f"(user_id={best_id}, cos={best_score:.3f} ≥ "
                        f"{threshold:.2f}); delete it first if you want to re-enrol",
                    )

            self.data[user_id] = {
                "name": user_name,
                "embedding": [float(x) for x in embedding.tolist()],
            }
            try:
                self._save_locked()
            except Exception:
                del self.data[user_id]
                raise
            log.info(
                "Enrolled %s (%s); embedding_dim=%d",
                user_id, user_name, len(embedding),
            )
            return True, ""

    def identify(self, query_emb: np.ndarray, threshold: float):
        with self._lock:
            return self._identify_locked(query_emb, threshold)

    def list_users(self) -> list[tuple[str, str]]:
        with self._lock:
            return [(uid, entry.get("name", "")) for uid, entry in self.data.items()]

    def __len__(self) -> int:
        with self._lock:
            return len(self.data)

    def delete(self, user_id: str) -> bool:
        # Idempotent — returning True on an absent user_id matches what
        # callers want (delete-then-re-enrol flows shouldn't error if a
        # stale id was already gone). Only an existing row triggers a disk
        # write, avoiding spurious enrolled.json rewrites.
        with self._lock:
            if user_id in self.data:
                existing = self.data[user_id]
                del self.data[user_id]
                try:
                    self._save_locked()
                except Exception:
                    self.data[user_id] = existing
                    raise
                log.info("Deleted enrolled user %s", user_id)
            else:
                log.info("Delete: user %s not enrolled — no-op", user_id)
        return True


# ---------------------------------------------------------------------------
# Module-level state populated by lifecycle hooks.
# ---------------------------------------------------------------------------

_engine: EcapaTdnnEngine | None = None
_db: EnrolledDB | None = None
_threshold_value: float = _DEFAULT_THRESHOLD
_configured_device: str | None = None
_applied_config: tuple[Path, float, str | None] | None = None
_runtime_lock = threading.RLock()


def _configure(cfg: dict) -> None:
    """Validate config and open the enrollment DB without loading the model."""
    global _db, _threshold_value, _configured_device, _applied_config
    threshold_raw = (
        cfg["threshold"]
        if "threshold" in cfg
        else os.environ.get("VOICEPRINT_THRESHOLD", _DEFAULT_THRESHOLD)
    )
    threshold = _validate_threshold(threshold_raw)
    data_dir = Path(cfg.get("data_dir", str(_data_dir())))
    device_raw = (
        cfg["device"]
        if "device" in cfg
        else os.environ.get("VOICEPRINT_DEVICE")
    )
    if device_raw is None:
        device = None
    elif not isinstance(device_raw, str) or not device_raw.strip():
        raise ValueError("device must be a non-empty string or null")
    else:
        device = device_raw.strip()
    resolved = (data_dir, threshold, device)

    with _runtime_lock:
        if _db is not None:
            if resolved == _applied_config:
                log.info("voiceprint init: already configured")
                return
            raise RuntimeError("voiceprint is already initialized with different configuration")

        data_dir.mkdir(parents=True, exist_ok=True)
        db = EnrolledDB(data_dir / "enrolled.json")
        _db = db
        _threshold_value = threshold
        _configured_device = device
        _applied_config = resolved
        log.info(
            "voiceprint init complete: data_dir=%s threshold=%.2f enrolled=%d",
            data_dir,
            threshold,
            len(db),
        )


def _release_engine() -> None:
    """Detach and close the current model while no inference is running."""
    global _engine
    with _runtime_lock:
        engine = _engine
        _engine = None
        if engine is not None:
            engine.close()


# ---------------------------------------------------------------------------
# gRPC servicers — one per atlas-registered contract, all sharing _engine + _db.
#
# Each Servicer subclasses a codegen-generated base class. Method names mirror
# the .srv filename (PascalCase), and the request/response types live in the
# generated pb module as `<ContractPascal>_Request` / `_Response`.
# ---------------------------------------------------------------------------


class _IdentifyServicer(pb_grpc.RobonixServiceVoiceprintIdentifyServicer):
    """robonix/service/voiceprint/identify — Identify(audio) → (user_id, confidence)."""

    def Identify(self, request, context):  # noqa: N802 (gRPC method)
        try:
            with _runtime_lock:
                engine = _engine
                db = _db
                threshold = _threshold_value
                if engine is None or db is None:
                    return vp.Identify_Response(
                        user_id="",
                        user_name="",
                        confidence=0.0,
                        is_known=False,
                        error=(
                            "voiceprint engine not active "
                            "(Driver(CMD_ACTIVATE) not received yet)"
                        ),
                    )
                emb = engine.extract_from_pcm(
                    request.audio_data,
                    request.encoding or "pcm_s16le",
                    request.sample_rate_hz or 16000,
                )
            uid, name, score, is_known = db.identify(emb, threshold)
            log.info(
                "Identify: best=%s (%s) score=%.4f is_known=%s",
                uid, name, score, is_known,
            )
            return vp.Identify_Response(
                user_id=uid, user_name=name, confidence=score,
                is_known=is_known, error="",
            )
        except Exception as exc:  # noqa: BLE001
            log.exception("Identify failed")
            return vp.Identify_Response(
                user_id="", user_name="", confidence=0.0,
                is_known=False, error=str(exc),
            )


class _EnrollServicer(pb_grpc.RobonixServiceVoiceprintEnrollServicer):
    """robonix/service/voiceprint/enroll — Enroll(audio + user_id + user_name)."""

    def Enroll(self, request, context):  # noqa: N802
        if not request.user_id or not request.audio_data:
            return vp.Enroll_Response(
                success=False, error="user_id and audio_data are required",
            )
        try:
            with _runtime_lock:
                engine = _engine
                db = _db
                threshold = _threshold_value
                if engine is None or db is None:
                    return vp.Enroll_Response(
                        success=False,
                        error=(
                            "voiceprint engine not active "
                            "(Driver(CMD_ACTIVATE) not received yet)"
                        ),
                    )
                emb = engine.extract_from_pcm(
                    request.audio_data,
                    request.encoding or "pcm_s16le",
                    request.sample_rate_hz or 16000,
                )

            # ID, display-name, and voice duplicate checks must share the
            # database lock with mutation + persistence. Otherwise two
            # concurrent requests can both pass the checks and overwrite the
            # same temporary file or persist duplicate speakers.
            success, error = db.enroll_unique(
                request.user_id,
                request.user_name,
                emb,
                threshold,
            )
            return vp.Enroll_Response(success=success, error=error)
        except Exception as exc:  # noqa: BLE001
            log.exception("Enroll failed")
            return vp.Enroll_Response(success=False, error=str(exc))


class _DeleteServicer(pb_grpc.RobonixServiceVoiceprintDeleteServicer):
    """robonix/service/voiceprint/delete — DeleteEnrolled(user_id)."""

    def DeleteEnrolled(self, request, context):  # noqa: N802
        if _db is None:
            return vp.DeleteEnrolled_Response(
                success=False, error="db not initialised",
            )
        if not request.user_id:
            return vp.DeleteEnrolled_Response(
                success=False, error="user_id is required",
            )
        try:
            _db.delete(request.user_id)
            return vp.DeleteEnrolled_Response(success=True, error="")
        except Exception as exc:  # noqa: BLE001
            log.exception("DeleteEnrolled failed")
            return vp.DeleteEnrolled_Response(success=False, error=str(exc))


class _ListServicer(pb_grpc.RobonixServiceVoiceprintListServicer):
    """robonix/service/voiceprint/list — ListEnrolled() → JSON catalog."""

    def ListEnrolled(self, request, context):  # noqa: N802
        if _db is None:
            return vp.ListEnrolled_Response(
                users_json="[]", count=0, error="db not initialised",
            )
        try:
            users = [{"user_id": uid, "user_name": name} for uid, name in _db.list_users()]
            return vp.ListEnrolled_Response(
                users_json=json.dumps(users, ensure_ascii=False),
                count=len(users), error="",
            )
        except Exception as exc:  # noqa: BLE001
            log.exception("ListEnrolled failed")
            return vp.ListEnrolled_Response(
                users_json="[]", count=0, error=str(exc),
            )


# ---------------------------------------------------------------------------
# robonix-api Service wiring
# ---------------------------------------------------------------------------

voiceprint = Service(id="voiceprint", namespace="robonix/service/voiceprint")

voiceprint.attach_grpc_servicer(
    "robonix/service/voiceprint/identify", _IdentifyServicer(),
)
voiceprint.attach_grpc_servicer(
    "robonix/service/voiceprint/enroll", _EnrollServicer(),
)
voiceprint.attach_grpc_servicer(
    "robonix/service/voiceprint/list", _ListServicer(),
)
voiceprint.attach_grpc_servicer(
    "robonix/service/voiceprint/delete", _DeleteServicer(),
)


@voiceprint.mcp("robonix/service/voiceprint/list")
def list_enrolled(req: ListEnrolled_Request) -> ListEnrolled_Response:
    """List enrolled voiceprints through the planner-visible MCP surface."""
    if _db is None:
        return ListEnrolled_Response(users_json="[]", count=0, error="db not initialised")
    try:
        users = [{"user_id": uid, "user_name": name} for uid, name in _db.list_users()]
        return ListEnrolled_Response(
            users_json=json.dumps(users, ensure_ascii=False),
            count=len(users),
            error="",
        )
    except Exception as exc:  # noqa: BLE001
        log.exception("ListEnrolled MCP failed")
        return ListEnrolled_Response(users_json="[]", count=0, error=str(exc))


@voiceprint.on_init
def init(cfg: dict):
    """Validate config and open the enrollment DB without hot model resources."""
    try:
        _configure(cfg)
        return Ok()
    except Exception as exc:  # noqa: BLE001
        log.exception("voiceprint init failed")
        return Err(f"voiceprint init failed: {type(exc).__name__}: {exc}")


@voiceprint.on_activate
def activate():
    """Load the ECAPA-TDNN model when the provider becomes active."""
    global _engine
    with _runtime_lock:
        if _db is None or _applied_config is None:
            return Err("voiceprint has not been initialized")
        if _engine is not None:
            return Ok()
        try:
            _engine = EcapaTdnnEngine(device=_configured_device)
            log.info("voiceprint activation complete")
            return Ok()
        except Exception as exc:  # noqa: BLE001
            _engine = None
            log.exception("voiceprint activation failed")
            return Err(f"voiceprint activation failed: {type(exc).__name__}: {exc}")


@voiceprint.on_deactivate
def deactivate():
    """Release the hot model while retaining config and enrollment state."""
    try:
        _release_engine()
        log.info("voiceprint deactivation complete")
        return Ok()
    except Exception as exc:  # noqa: BLE001
        log.exception("voiceprint deactivation failed")
        return Err(f"voiceprint deactivation failed: {type(exc).__name__}: {exc}")


@voiceprint.on_shutdown
def shutdown():
    """Release the model on direct shutdown as well as normal deactivation."""
    try:
        _release_engine()
        return Ok()
    except Exception as exc:  # noqa: BLE001
        log.exception("voiceprint shutdown cleanup failed")
        return Err(f"voiceprint shutdown failed: {type(exc).__name__}: {exc}")


def main() -> int:
    voiceprint.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
