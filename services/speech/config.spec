# Runtime configuration accepted by the Speech service.
#
# This documents the mapping passed as the service instance's `config:` value.
# It is not loaded as a schema. Credentials remain environment variables and
# must not be committed to a robot deployment manifest.

config:
  # string, default: local; accepted: local, tencent, mock, custom.
  # Selects the ASR/TTS backend family initialized by the service.
  speech_backend: local

  # string provider id, default: empty.
  # Speaker primitive used by speak requests that do not specify a target.
  default_speaker_provider_id: ""

  # boolean, default: false.
  # Disable one-shot Whisper initialization in the local backend. Streaming
  # FunASR and TTS can remain available.
  disable_whisper: false

  # list of non-empty strings; defaults to the Mandarin wake phrase below.
  # Wake phrases compiled into the runtime keyword file.
  wake_words: ["罗伯特"]  # i18n-ok: Mandarin runtime default

  # string path or null, default: bundled build artifact.
  # Directory containing the sherpa-onnx keyword-spotting model.
  wake_word_model_dir: null

  # string path or null, default: generated from wake_words.
  # Prebuilt sherpa-onnx keywords file. Set only when managing it externally.
  wake_word_keywords_file: null

  # float, default: 2.0; must be positive.
  # Decoder score boost applied while generating wake-word keywords.
  wake_word_boost: 2.0

  # float probability-like score, default: 0.45; valid range: 0.0 < value <= 1.0.
  # Minimum wake-word detection score.
  wake_word_threshold: 0.45

  # integer threads, default: 2; minimum effective value: 1.
  # CPU threads used by the wake-word backend.
  wake_word_num_threads: 2

  # list of non-empty strings; defaults to the Mandarin warm-up phrase below.
  # Short phrases synthesized during initialization to warm the TTS backend.
  tts_warm_phrases: ["我在"]  # i18n-ok: Mandarin runtime default

  # Local backend ----------------------------------------------------------

  # string model id or path, default: openai/whisper-large-v3.
  # One-shot Whisper model used by the local backend.
  asr_model: openai/whisper-large-v3

  # string device, default: cuda.
  # Compute device passed to the Whisper pipeline, for example cuda or cpu.
  asr_device: cuda

  # float seconds, default: 30.0; must be positive.
  # Audio chunk length used by one-shot Whisper inference.
  asr_chunk_length: 30.0

  # integer, default: 4; must be positive.
  # Whisper inference batch size; lower it when GPU memory is constrained.
  asr_batch_size: 4

  # string model id or path, default: paraformer-zh-streaming.
  # FunASR model used for streaming recognition in the local backend.
  funasr_model: paraformer-zh-streaming

  # string device, default: auto; accepted by FunASR: auto, cpu, cuda.
  # Auto uses CUDA when available and otherwise falls back to CPU.
  funasr_device: auto

  # list of three integers, default: [0, 10, 5].
  # FunASR streaming chunk configuration in the model's native format.
  funasr_chunk_size: [0, 10, 5]

  # string Edge TTS voice, default: zh-CN-XiaoxiaoNeural.
  # Voice selected by the local TTS backend when a request has no override.
  tts_voice: zh-CN-XiaoxiaoNeural

  # Tencent backend --------------------------------------------------------

  # string numeric application id, no default.
  # Tencent ASR AppID. Secret ID and Secret Key remain in
  # TENCENTCLOUD_SECRET_ID and TENCENTCLOUD_SECRET_KEY environment variables.
  tencent_asr_appid: ""

  # string engine id, default: 16k_zh.
  # 16k_zh uses the basic real-time Mandarin product. 16k_zh_en is a
  # separately billed large-model engine and does not consume its free quota.
  tencent_asr_engine: 16k_zh

  # string hostname, default: asr.cloud.tencent.com.
  # Tencent signed WebSocket host. Normally leave unchanged.
  tencent_asr_host: asr.cloud.tencent.com

  # integer Tencent voice type, default: 1001.
  # The selected voice must be enabled for the Tencent account/product.
  tencent_tts_voice_type: 1001

  # string Tencent region, default: ap-guangzhou.
  # Region sent to the TextToVoice API.
  tencent_tts_region: ap-guangzhou

  # integer Tencent model type, default: 1.
  # Tencent TextToVoice model selector; availability depends on the voice.
  tencent_tts_model_type: 1

  # integer hertz, default: 16000; supported values depend on Tencent TTS.
  # Output rate advertised to downstream speaker primitives.
  tencent_tts_sample_rate: 16000

  # string codec, default: pcm.
  # Keep pcm for the Robonix 16-bit PCM speaker path.
  tencent_tts_codec: pcm

  # integer language selector, default: 1.
  # Tencent TextToVoice PrimaryLanguage request value.
  tencent_tts_primary_language: 1

  # Custom backend ---------------------------------------------------------

  # string module:Class or empty, default: empty.
  # One-shot ASR implementation loaded when speech_backend is custom.
  speech_asr_backend_class: ""

  # string module:Class or empty, default: speech_asr_backend_class.
  # Streaming ASR implementation loaded for custom mode.
  speech_asr_stream_backend_class: ""

  # string module:Class or empty, default: empty.
  # TTS implementation loaded for custom mode.
  speech_tts_backend_class: ""
