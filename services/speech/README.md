# speech_service

Robonix speech service for `robonix/service/speech/*`. It exposes ASR, streaming
ASR, TTS, streaming TTS, dialog state, and speaker helper capabilities.

Backends:

- `local` (default): Whisper one-shot ASR, FunASR streaming ASR, Edge TTS.
- `tencent`: Tencent Cloud real-time ASR WebSocket and TextToVoice TTS API.
- `custom`: load ASR/TTS backend classes from `module:Class` environment
  variables without changing Liaison or speech capability contracts.

Tencent Cloud mode:

1. Open the Tencent Cloud ASR console and activate Automatic Speech Recognition.
2. Open CAM / Access Management -> Access Key -> API Key Management and create
   an API key. Save `SecretKey` immediately; Tencent Cloud only shows it when
   the key is created. The same page/account information provides the AppID.
3. Select the backend in the deployment manifest so `rbnx build` also knows
   that no local model or GPU dependencies are needed:

```yaml
env:
  SPEECH_BACKEND: tencent

service:
  - name: speech
    path: ${ROBONIX_SOURCE_PATH}/services/speech
    config:
      speech_backend: tencent
      tencent_asr_appid: "1234567890"
      tencent_asr_engine: 16k_zh
      tencent_tts_voice_type: 502003
      tencent_tts_region: ap-guangzhou
```

4. Export only the credentials on the operator machine before `rbnx boot`:

```bash
export TENCENTCLOUD_SECRET_ID=...
export TENCENTCLOUD_SECRET_KEY=...
```

Do not commit Tencent credentials. Put them in the operator shell, a local
ignored env file, or a machine-local boot wrapper. The AppID and non-secret
backend settings belong in the deployment manifest. Useful optional knobs:
`TENCENT_ASR_ENGINE` (default `16k_zh`), `TENCENT_TTS_VOICE_TYPE` (default
`1001`), and `TENCENT_TTS_REGION` (default `ap-guangzhou`).

With `SPEECH_BACKEND=tencent`, the build installs only the cloud client and
audio adaptation dependencies. It does not install or warm FunASR, Whisper,
Torch, CUDA, or Edge TTS. The `local` backend keeps those dependencies in the
`local` optional dependency set and preloads its models during build.

The TTS contract returns 16 kHz mono `pcm_s16le` bytes in both local and Tencent
modes so existing audio speaker primitives can play the response directly.

## Wake word

Wake-word recognition is part of the Speech package, not Liaison. The default
phrase is the bundled Mandarin wake phrase (YAML `\u7f57\u4f2f\u7279`); it is
selected because the bundled KWS model recognizes it from a 16 kHz Tencent TTS
test stream. A deployment can replace it without
changing code:

```yaml
service:
  - name: speech
    config:
      wake_words: ["\u7f57\u4f2f\u7279"]
      wake_word_boost: 2.0
      wake_word_threshold: 0.45
```

`SPEECH_WAKE_WORDS` is the environment equivalent (comma-separated). The
Speech build downloads the KWS model; runtime does not download model weights.

## Adding a new ASR/TTS algorithm

Liaison depends only on the speech service capabilities:

- `robonix/service/speech/asr`
- `robonix/service/speech/asr_stream`
- `robonix/service/speech/tts`
- `robonix/service/speech/tts_stream`

Algorithm code stays inside this package. To add a new backend, create a new
file such as `services/speech/speech_service/my_asr.py` and implement the
backend method shape you need:

```python
from collections.abc import Iterable, Iterator


class MyASRBackend:
    def recognize(self, audio_bytes: bytes, encoding: str, sample_rate: int, language: str) -> dict:
        return {"text": "...", "confidence": 0.9}

    def recognize_stream(self, pcm_chunks: Iterable[bytes]) -> Iterator[dict]:
        for chunk in pcm_chunks:
            # Forward chunk to your local model, cloud API, or gRPC stub.
            yield {
                "event_type": 0,
                "text": "...",
                "confidence": 0.9,
                "is_final": False,
            }


class MyTTSBackend:
    async def synthesize(self, text: str, voice: str = "", speed: float = 1.0) -> bytes:
        return b""  # 16 kHz mono pcm_s16le

    async def synthesize_stream(self, text: str, voice: str = "", speed: float = 1.0):
        yield await self.synthesize(text, voice, speed)
```

Then select it at boot:

```bash
export SPEECH_BACKEND=custom
export SPEECH_ASR_BACKEND_CLASS=speech_service.my_asr:MyASRBackend
export SPEECH_ASR_STREAM_BACKEND_CLASS=speech_service.my_asr:MyASRBackend
export SPEECH_TTS_BACKEND_CLASS=speech_service.my_tts:MyTTSBackend
```

The backend receives already-adapted ASR audio as 16 kHz mono `pcm_s16le`.
Streaming ASR yields event dictionaries; one-shot ASR returns a dictionary with
`text` and `confidence`. TTS must return or yield 16 kHz mono `pcm_s16le` bytes.

The backend may call any implementation internally: a local model, a cloud API,
or an existing gRPC stub. Do not change Liaison for algorithm swaps.
