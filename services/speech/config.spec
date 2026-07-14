# Speech service runtime config. Only select a backend and configure values
# that differ from that backend's defaults. Credentials remain environment
# variables and are never stored in the deployment manifest.

config:
  speech_backend: local       # local | tencent | mock | custom
  default_speaker_provider_id: ""
  wake_words: ["罗伯特"]
  disable_whisper: false

  # Local backend
  funasr_device: auto
  tts_voice: zh-CN-XiaoxiaoNeural

  # Tencent backend. TENCENTCLOUD_SECRET_ID and
  # TENCENTCLOUD_SECRET_KEY are environment variables.
  tencent_asr_appid: ""
  tencent_asr_engine: 16k_zh
  tencent_tts_voice_type: 1001
  tencent_tts_region: ap-guangzhou
  tencent_tts_model_type: 1
  tencent_tts_sample_rate: 16000
  tencent_tts_codec: pcm
