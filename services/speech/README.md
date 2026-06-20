# speech_service

Robonix speech service for `robonix/service/speech/*`. It exposes ASR, streaming
ASR, TTS, streaming TTS, dialog state, and speaker helper capabilities.

Backends:

- `local` (default): Whisper one-shot ASR, FunASR streaming ASR, Edge TTS.
- `tencent`: Tencent Cloud real-time ASR WebSocket and TextToVoice TTS API.

Tencent Cloud mode:

```bash
export SPEECH_BACKEND=tencent
export TENCENT_ASR_APPID=...
export TENCENTCLOUD_SECRET_ID=...
export TENCENTCLOUD_SECRET_KEY=...
```

Do not commit Tencent credentials. Put them in the operator shell, a local
ignored env file, or a machine-local boot wrapper. Useful optional knobs:
`TENCENT_ASR_ENGINE` (default `16k_zh_en`), `TENCENT_TTS_VOICE_TYPE` (default
`1001`), and `TENCENT_TTS_REGION` (default `ap-guangzhou`).

The TTS contract returns 16 kHz mono `pcm_s16le` bytes in both local and Tencent
modes so existing audio speaker primitives can play the response directly.
