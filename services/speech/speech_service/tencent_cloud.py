"""Tencent Cloud speech backends for Robonix speech_service.

The backends keep the existing speech_service contracts intact while moving
ASR/TTS inference out of the local process. Credentials are read only from
environment variables or runtime config mapped into env; never commit them.
"""

from __future__ import annotations

import asyncio
import base64
import hashlib
import hmac
import json
import logging
import os
import random
import time
import uuid
from dataclasses import dataclass
from typing import Iterator
from urllib.parse import quote, urlencode

import requests
from websockets.sync.client import connect

log = logging.getLogger(__name__)


def _env_first(*names: str, default: str = "") -> str:
    for name in names:
        value = os.environ.get(name, "").strip()
        if value:
            return value
    return default


def _require_env(*names: str) -> str:
    value = _env_first(*names)
    if value:
        return value
    joined = " / ".join(names)
    raise RuntimeError(f"missing Tencent Cloud credential env: {joined}")


@dataclass(frozen=True)
class TencentCredentials:
    appid: str
    secret_id: str
    secret_key: str

    @classmethod
    def from_env(cls) -> "TencentCredentials":
        return cls(
            appid=_require_env("TENCENT_ASR_APPID", "TENCENTCLOUD_APPID"),
            secret_id=_require_env("TENCENTCLOUD_SECRET_ID", "TENCENT_SECRET_ID"),
            secret_key=_require_env("TENCENTCLOUD_SECRET_KEY", "TENCENT_SECRET_KEY"),
        )


class TencentRealtimeASRBackend:
    """Tencent Cloud real-time ASR over WebSocket.

    The gRPC servicer still receives chunked 16 kHz mono pcm_s16le audio. For
    each RecognizeStream call we open one Tencent WebSocket session, forward
    audio at near-real-time pace, then emit Tencent's stable slice results as
    Robonix ASR events.
    """

    def __init__(self):
        self.creds = TencentCredentials.from_env()
        self.engine = os.environ.get("TENCENT_ASR_ENGINE", "16k_zh")
        self.host = os.environ.get("TENCENT_ASR_HOST", "asr.cloud.tencent.com")
        self.needvad = int(os.environ.get("TENCENT_ASR_NEEDVAD", "1"))
        self.voice_format = int(os.environ.get("TENCENT_ASR_VOICE_FORMAT", "1"))
        self.filter_dirty = int(os.environ.get("TENCENT_ASR_FILTER_DIRTY", "0"))
        self.filter_modal = int(os.environ.get("TENCENT_ASR_FILTER_MODAL", "0"))
        self.filter_punc = int(os.environ.get("TENCENT_ASR_FILTER_PUNC", "0"))
        self.chunk_bytes = int(os.environ.get("TENCENT_ASR_CHUNK_BYTES", "6400"))
        self.recv_timeout_s = float(os.environ.get("TENCENT_ASR_RECV_TIMEOUT", "0.05"))
        log.info(
            "Tencent ASR initialized (appid=%s engine=%s credential=%s)",
            self.creds.appid,
            self.engine,
            self._credential_fingerprint(self.creds),
        )

    def recognize(
        self,
        audio_data: bytes,
        encoding: str = "pcm_s16le",
        sample_rate: int = 16000,
        language: str = "",
    ) -> dict:
        """Recognize a complete utterance by reusing the Tencent stream API."""
        _ = (encoding, sample_rate, language)
        transcript = ""
        confidence = 0.0
        for event in self.recognize_stream(self._chunk_pcm(audio_data)):
            text = event.get("text", "")
            if text:
                transcript += text
                confidence = float(event.get("confidence", confidence))
        return {"text": transcript.strip(), "confidence": confidence}

    def recognize_stream(self, pcm_chunks: Iterator[bytes]) -> Iterator[dict]:
        url = self._signed_url()
        last_text = ""
        # Robot deployments may export a proxy for GitHub/model downloads.
        # Tencent's signed ASR WebSocket must connect directly: inheriting
        # HTTP_PROXY/ALL_PROXY can route the handshake through a local proxy
        # and Tencent then reports a misleading resource-package error (4004).
        with connect(
            url,
            max_size=None,
            open_timeout=5,
            close_timeout=2,
            proxy=None,
        ) as ws:
            first = self._recv_json(ws, timeout_s=5.0)
            if first and first.get("code", 0) != 0:
                raise RuntimeError(f"Tencent ASR handshake failed: {first}")

            frame_buf = bytearray()
            for chunk in pcm_chunks:
                if not chunk:
                    continue
                frame_buf.extend(chunk)
                while len(frame_buf) >= self.chunk_bytes:
                    frame = bytes(frame_buf[: self.chunk_bytes])
                    del frame_buf[: self.chunk_bytes]
                    ws.send(frame)
                    for event in self._drain_results(ws):
                        event, last_text = self._to_incremental_event(event, last_text)
                        if event:
                            yield event
                    time.sleep(len(frame) / 2 / 16000)

            if frame_buf:
                ws.send(bytes(frame_buf))
                for event in self._drain_results(ws):
                    event, last_text = self._to_incremental_event(event, last_text)
                    if event:
                        yield event

            ws.send(json.dumps({"type": "end"}))
            while True:
                msg = self._recv_json(ws, timeout_s=5.0)
                if msg is None:
                    break
                for event in self._event_from_message(msg):
                    event, last_text = self._to_incremental_event(event, last_text)
                    if event:
                        yield event
                if msg.get("final") == 1:
                    break

    def _signed_url(self) -> str:
        # Driver(INIT) is allowed to update the manifest-backed Tencent
        # settings after the service process has started. Refresh the session
        # snapshot here so a long-lived backend never signs with stale AppID
        # or credentials captured by an earlier initialization attempt.
        self.creds = TencentCredentials.from_env()
        self.engine = os.environ.get("TENCENT_ASR_ENGINE", "16k_zh")
        now = int(time.time())
        params = {
            "engine_model_type": self.engine,
            "expired": now + 24 * 3600,
            "filter_dirty": self.filter_dirty,
            "filter_modal": self.filter_modal,
            "filter_punc": self.filter_punc,
            "needvad": self.needvad,
            "nonce": random.randint(1, 2_147_483_647),
            "secretid": self.creds.secret_id,
            "timestamp": now,
            "voice_format": self.voice_format,
            "voice_id": str(uuid.uuid4()),
        }
        query = "&".join(f"{k}={params[k]}" for k in sorted(params))
        sign_src = f"{self.host}/asr/v2/{self.creds.appid}?{query}"
        digest = hmac.new(
            self.creds.secret_key.encode("utf-8"),
            sign_src.encode("utf-8"),
            hashlib.sha1,
        ).digest()
        signature = base64.b64encode(digest).decode("utf-8")
        return (
            f"wss://{self.host}/asr/v2/{self.creds.appid}?"
            f"{query}&signature={quote(signature, safe='')}"
        )

    @staticmethod
    def _credential_fingerprint(creds: TencentCredentials) -> str:
        material = f"{creds.appid}\0{creds.secret_id}".encode("utf-8")
        return hashlib.sha256(material).hexdigest()[:12]

    def _chunk_pcm(self, audio_data: bytes) -> Iterator[bytes]:
        for i in range(0, len(audio_data), self.chunk_bytes):
            yield audio_data[i : i + self.chunk_bytes]

    def _drain_results(self, ws) -> Iterator[dict]:
        while True:
            msg = self._recv_json(ws, timeout_s=self.recv_timeout_s)
            if msg is None:
                break
            yield from self._event_from_message(msg)

    @staticmethod
    def _recv_json(ws, timeout_s: float):
        try:
            raw = ws.recv(timeout=timeout_s)
        except TimeoutError:
            return None
        if isinstance(raw, bytes):
            raw = raw.decode("utf-8", errors="replace")
        return json.loads(raw)

    @staticmethod
    def _event_from_message(msg: dict) -> Iterator[dict]:
        code = int(msg.get("code", 0))
        if code != 0:
            raise RuntimeError(msg.get("message") or f"Tencent ASR error code={code}")
        result = msg.get("result") or {}
        text = result.get("voice_text_str") or result.get("text") or ""
        if not text:
            return
        slice_type = int(result.get("slice_type", 1))
        yield {
            "text": text,
            "confidence": 0.9,
            "is_final": msg.get("final") == 1 or slice_type == 2,
            "event_type": 1 if msg.get("final") == 1 or slice_type == 2 else 0,
        }

    @staticmethod
    def _to_incremental_event(event: dict, last_text: str) -> tuple[dict | None, str]:
        """Hold Tencent's revisable hypothesis until it becomes final.

        Tencent sends cumulative snapshots and may rewrite earlier characters
        between snapshots.  Liaison's stream contract is append-only, so a
        revised snapshot cannot be represented as a safe delta (for example
        ``I may`` may become ``I have already...``). Emitting partials would
        concatenate every revision.  Keep only the newest snapshot and emit it
        once when Tencent marks it final.
        """
        text = event.get("text", "")
        if not text:
            return None, last_text
        last_text = text
        if not event.get("is_final", False):
            return None, last_text
        out = dict(event)
        out["text"] = last_text
        return out, last_text


class TencentTTSBackend:
    """Tencent Cloud TextToVoice backend returning 16 kHz mono PCM bytes."""

    service = "tts"
    host = "tts.tencentcloudapi.com"
    endpoint = "https://tts.tencentcloudapi.com"
    version = "2019-08-23"
    action = "TextToVoice"

    def __init__(self):
        self.secret_id = _require_env("TENCENTCLOUD_SECRET_ID", "TENCENT_SECRET_ID")
        self.secret_key = _require_env("TENCENTCLOUD_SECRET_KEY", "TENCENT_SECRET_KEY")
        self.region = os.environ.get("TENCENT_TTS_REGION", "ap-guangzhou")
        self.voice_type = int(os.environ.get("TENCENT_TTS_VOICE_TYPE", "1001"))
        self.model_type = int(os.environ.get("TENCENT_TTS_MODEL_TYPE", "1"))
        self.sample_rate = int(os.environ.get("TENCENT_TTS_SAMPLE_RATE", "16000"))
        self.codec = os.environ.get("TENCENT_TTS_CODEC", "pcm")
        self.primary_language = int(os.environ.get("TENCENT_TTS_PRIMARY_LANGUAGE", "1"))
        log.info(
            "Tencent TTS initialized (voice_type=%s, sample_rate=%s, codec=%s)",
            self.voice_type,
            self.sample_rate,
            self.codec,
        )

    async def synthesize(self, text: str, voice: str = "", speed: float = 1.0) -> bytes:
        primary_language = self._primary_language_for_text(text)
        payload = {
            "Text": text,
            "SessionId": str(uuid.uuid4()),
            "Volume": 0,
            "Speed": self._speed_to_tencent(speed),
            "ProjectId": 0,
            "ModelType": self.model_type,
            "VoiceType": int(voice) if voice else self.voice_type,
            "PrimaryLanguage": primary_language,
            "SampleRate": self.sample_rate,
            "Codec": self.codec,
            "EnableSubtitle": False,
        }
        log.info("Tencent TTS request: voice_type=%s primary_language=%s text_len=%s", payload["VoiceType"], primary_language, len(text or ""))
        response = await asyncio.to_thread(self._post_json, payload)
        audio_b64 = response.get("Audio", "")
        if not audio_b64:
            raise RuntimeError(f"Tencent TTS returned no audio: {response}")
        return base64.b64decode(audio_b64)

    async def synthesize_stream(self, text: str, voice: str = "", speed: float = 1.0):
        data = await self.synthesize(text, voice, speed)
        for i in range(0, len(data), 4096):
            yield data[i : i + 4096]

    def _primary_language_for_text(self, text: str) -> int:
        override = os.environ.get("TENCENT_TTS_PRIMARY_LANGUAGE", "").strip()
        if override:
            return int(override)
        letters = sum(1 for ch in text if ("A" <= ch <= "Z") or ("a" <= ch <= "z"))
        cjk = sum(1 for ch in text if "\u4e00" <= ch <= "\u9fff")
        # Tencent TextToVoice exposes 1=Chinese, 2=English. Use English only
        # when the utterance is primarily English; mixed Chinese/English sounds
        # more consistent with Chinese as the primary language.
        return 2 if letters > 0 and cjk == 0 else 1

    @staticmethod
    def _speed_to_tencent(speed: float) -> float:
        if speed <= 0:
            return 0
        if speed < 0.7:
            return -2
        if speed < 0.9:
            return -1
        if speed < 1.1:
            return 0
        if speed < 1.35:
            return 1
        if speed < 1.75:
            return 2
        return min(6, round((speed - 1.0) * 2))

    def _post_json(self, payload: dict) -> dict:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        timestamp = int(time.time())
        headers = self._headers(body, timestamp)
        resp = requests.post(self.endpoint, data=body.encode("utf-8"), headers=headers, timeout=10)
        resp.raise_for_status()
        data = resp.json()
        response = data.get("Response", {})
        if "Error" in response:
            err = response["Error"]
            raise RuntimeError(f"Tencent TTS {err.get('Code')}: {err.get('Message')}")
        return response

    def _headers(self, body: str, timestamp: int) -> dict:
        date = time.strftime("%Y-%m-%d", time.gmtime(timestamp))
        hashed_request_payload = hashlib.sha256(body.encode("utf-8")).hexdigest()
        canonical_headers = f"content-type:application/json; charset=utf-8\nhost:{self.host}\n"
        signed_headers = "content-type;host"
        canonical_request = "\n".join(
            [
                "POST",
                "/",
                "",
                canonical_headers,
                signed_headers,
                hashed_request_payload,
            ]
        )
        credential_scope = f"{date}/{self.service}/tc3_request"
        hashed_canonical_request = hashlib.sha256(
            canonical_request.encode("utf-8")
        ).hexdigest()
        string_to_sign = "\n".join(
            [
                "TC3-HMAC-SHA256",
                str(timestamp),
                credential_scope,
                hashed_canonical_request,
            ]
        )
        secret_date = _sign(("TC3" + self.secret_key).encode("utf-8"), date)
        secret_service = _sign(secret_date, self.service)
        secret_signing = _sign(secret_service, "tc3_request")
        signature = hmac.new(
            secret_signing, string_to_sign.encode("utf-8"), hashlib.sha256
        ).hexdigest()
        authorization = (
            "TC3-HMAC-SHA256 "
            f"Credential={self.secret_id}/{credential_scope}, "
            f"SignedHeaders={signed_headers}, Signature={signature}"
        )
        return {
            "Authorization": authorization,
            "Content-Type": "application/json; charset=utf-8",
            "Host": self.host,
            "X-TC-Action": self.action,
            "X-TC-Timestamp": str(timestamp),
            "X-TC-Version": self.version,
            "X-TC-Region": self.region,
        }


def _sign(key: bytes, msg: str) -> bytes:
    return hmac.new(key, msg.encode("utf-8"), hashlib.sha256).digest()
