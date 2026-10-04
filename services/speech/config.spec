specVersion: 1
description: >-
  Backend selection, wake word and ASR/TTS engine settings for the Speech
  service. Tencent Cloud credentials are read only from environment variables
  and must not be committed to a robot deployment manifest.

# Most keys are copied into the environment variable named in their
# description, overriding a value exported before start; non-string values are
# JSON-encoded first. A key that is absent leaves that variable unchanged.
#
# Tencent credentials: TENCENTCLOUD_SECRET_ID (or TENCENT_SECRET_ID) and
# TENCENTCLOUD_SECRET_KEY (or TENCENT_SECRET_KEY). No config key accepts them.
#
# A backend that fails to construct (bad model, bad Tencent setting, missing
# credential) leaves its contract UNAVAILABLE; Driver(CMD_INIT) fails when
# ASR, streaming ASR and TTS all fail. A wake-word failure, including an
# invalid wake-word setting, only leaves the wake_word contract UNAVAILABLE.
properties:
  speech_backend:
    type: string
    description: >-
      ASR/TTS backend family: local (Whisper, FunASR, Edge TTS), tencent
      (Tencent Cloud ASR and TTS) or custom (classes named by the Custom
      backend keys). Environment fallback: SPEECH_BACKEND.
    enum: [local, tencent, custom]
    default: local
    # Mock backends are selected only by SPEECH_BACKEND=mock in the process
    # environment at start, which then overrides this key; speech_backend:
    # mock here runs the local backend.
    # rbnx build reads SPEECH_BACKEND, not this key, to choose which
    # dependencies to install, so set both for tencent or custom.

  default_speaker_provider_id:
    type: string
    x-provider: robonix/primitive/audio/speaker
    description: >-
      Speaker primitive used by speak requests that give no target. When
      empty, the first speaker provider found in Atlas is used. Environment
      fallback: SPEECH_DEFAULT_SPEAKER_PROVIDER_ID.
    default: ""

  disable_whisper:
    type: boolean
    description: >-
      Skip one-shot Whisper initialization in the local backend; the asr
      contract then returns UNAVAILABLE while streaming FunASR and TTS remain
      available. Environment fallback: SPEECH_DISABLE_WHISPER (1, true or
      yes).
    default: false

  wake_words:
    type: array
    items:
      type: string
    description: >-
      Wake phrases compiled into the keyword file. Phrases must not contain
      line breaks, ":", "@" or "#"; blank entries and duplicates are dropped.
      An empty list uses the default. Ignored when wake_word_keywords_file is
      set. Environment fallback: SPEECH_WAKE_WORDS (comma-separated or a JSON
      list).
    default: ["\u7f57\u4f2f\u7279"]  # Mandarin wake phrase

  wake_word_model_dir:
    type: [string, "null"]
    description: >-
      Directory of the sherpa-onnx keyword-spotting model. Null uses the
      model downloaded by the build into
      <package>/rbnx-build/models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20.
    default: null

  wake_word_keywords_file:
    type: [string, "null"]
    description: >-
      Prebuilt sherpa-onnx keywords file. Set only when managing it
      externally; null generates it from wake_words, wake_word_boost and
      wake_word_threshold.
    default: null

  wake_word_boost:
    type: number
    description: Decoder score boost written into the generated keywords file. Must be greater than 0.
    default: 2.0
    # 0 and null are replaced by the default.

  wake_word_threshold:
    type: number
    description: Minimum detection score written into the generated keywords file. Must be greater than 0.
    default: 0.45
    maximum: 1
    # 0 and null are replaced by the default.

  wake_word_num_threads:
    type: integer
    description: CPU threads used by the wake-word backend.
    default: 2
    minimum: 1
    # 0 and null are replaced by the default; negative values become 1.

  tts_warm_phrases:
    type: array
    items:
      type: string
    description: >-
      Short phrases synthesized during initialization to warm the TTS cache
      when a TTS backend is available. An empty list uses the default; a list
      whose entries are all blank fails Driver(CMD_INIT). Environment
      fallback: SPEECH_TTS_WARM_PHRASES (comma-separated or a JSON list).
    default: ["\u6211\u5728"]  # Mandarin warm-up phrase

  asr_model:
    type: string
    x-group: Local backend
    description: >-
      Whisper model ID or path for one-shot ASR. Loaded with local files
      only, so it must already be in the Hugging Face cache or on disk.
      Environment fallback: ASR_MODEL.
    default: openai/whisper-large-v3

  asr_device:
    type: string
    x-group: Local backend
    description: >-
      Device passed to the Whisper pipeline, for example cuda or cpu.
      Environment fallback: ASR_DEVICE.
    default: cuda

  asr_chunk_length:
    type: number
    x-group: Local backend
    description: >-
      Audio chunk length in seconds for one-shot Whisper inference.
      Environment fallback: ASR_CHUNK_LENGTH. Must be greater than 0.
    default: 30.0

  asr_batch_size:
    type: integer
    x-group: Local backend
    description: >-
      Whisper inference batch size; lower it when GPU memory is constrained.
      Environment fallback: ASR_BATCH_SIZE.
    default: 4
    minimum: 1

  funasr_model:
    type: string
    x-group: Local backend
    description: >-
      FunASR model for streaming recognition. Loaded with local files only.
      Environment fallback: FUNASR_MODEL.
    default: paraformer-zh-streaming

  funasr_device:
    type: string
    x-group: Local backend
    description: >-
      FunASR device. auto tries CUDA when available and falls back to CPU if
      loading fails; any other value, such as cpu or cuda, is the only device
      tried. Environment fallback: FUNASR_DEVICE.
    default: auto

  funasr_chunk_size:
    type: array
    items:
      type: integer
    x-group: Local backend
    description: >-
      FunASR streaming chunk configuration [left, current, right] in the
      model's native format. An environment value that is not valid JSON
      falls back to the default. Environment fallback: FUNASR_CHUNK_SIZE.
    default: [0, 10, 5]

  tts_voice:
    type: string
    x-group: Local backend
    description: >-
      Edge TTS voice used when a request has no voice override. Environment
      fallback: TTS_VOICE.
    default: zh-CN-XiaoxiaoNeural

  tencent_asr_appid:
    type: [string, integer]
    x-group: Tencent backend
    description: >-
      Tencent ASR AppID; required for Tencent ASR, not used by TTS. No
      default. Environment fallback: TENCENT_ASR_APPID, then
      TENCENTCLOUD_APPID.

  tencent_asr_engine:
    type: string
    x-group: Tencent backend
    description: >-
      ASR engine model type. 16k_zh uses the basic real-time Mandarin
      product. 16k_zh_en is a separately billed large-model engine and does
      not consume its free quota. Environment fallback: TENCENT_ASR_ENGINE.
    default: 16k_zh

  tencent_asr_host:
    type: string
    x-group: Tencent backend
    description: >-
      Hostname for the signed ASR WebSocket; normally left unchanged.
      Environment fallback: TENCENT_ASR_HOST.
    default: asr.cloud.tencent.com

  tencent_tts_voice_type:
    type: integer
    x-group: Tencent backend
    description: >-
      TextToVoice VoiceType. The voice must be enabled for the Tencent account
      and product. Environment fallback: TENCENT_TTS_VOICE_TYPE.
    default: 1001

  tencent_tts_region:
    type: string
    x-group: Tencent backend
    description: >-
      Region sent to the TextToVoice API. Environment fallback:
      TENCENT_TTS_REGION.
    default: ap-guangzhou

  tencent_tts_model_type:
    type: integer
    x-group: Tencent backend
    description: >-
      TextToVoice ModelType; availability depends on the voice. Environment
      fallback: TENCENT_TTS_MODEL_TYPE.
    default: 1

  tencent_tts_sample_rate:
    type: integer
    x-group: Tencent backend
    description: >-
      TTS output sample rate in hertz. Only 16000 is accepted, as required by
      the 16-bit PCM speaker path. Environment fallback:
      TENCENT_TTS_SAMPLE_RATE.
    enum: [16000]
    default: 16000

  tencent_tts_codec:
    type: string
    x-group: Tencent backend
    description: >-
      TTS output codec. Only pcm is accepted, as required by the 16-bit PCM
      speaker path. Environment fallback: TENCENT_TTS_CODEC.
    enum: [pcm]
    default: pcm

  tencent_tts_max_chars:
    type: integer
    x-group: Tencent backend
    description: >-
      Maximum characters per TextToVoice request; longer text is split at
      sentence punctuation, commas or whitespace. Environment fallback:
      TENCENT_TTS_MAX_CHARS.
    default: 140
    minimum: 1
    maximum: 150

  tencent_tts_max_total_chars:
    type: integer
    x-group: Tencent backend
    description: >-
      Maximum characters in one synthesis call; longer input is rejected.
      Environment fallback: TENCENT_TTS_MAX_TOTAL_CHARS.
    default: 5000
    minimum: 1
    maximum: 6000

  tencent_tts_primary_language:
    type: integer
    x-group: Tencent backend
    description: >-
      TextToVoice PrimaryLanguage: 1 for Chinese, 2 for English. Omit to
      choose per utterance: 2 when the text has ASCII letters and no CJK
      ideographs, otherwise 1. Environment fallback:
      TENCENT_TTS_PRIMARY_LANGUAGE.
    enum: [1, 2]

  speech_asr_backend_class:
    type: string
    x-group: Custom backend
    description: >-
      One-shot ASR class as module:Class, loaded when speech_backend is
      custom. Empty leaves the asr contract UNAVAILABLE. Environment
      fallback: SPEECH_ASR_BACKEND_CLASS.
    default: ""

  speech_asr_stream_backend_class:
    type: string
    x-group: Custom backend
    description: >-
      Streaming ASR class as module:Class for custom mode. Empty uses
      speech_asr_backend_class. Environment fallback:
      SPEECH_ASR_STREAM_BACKEND_CLASS.
    default: ""

  speech_tts_backend_class:
    type: string
    x-group: Custom backend
    description: >-
      TTS class as module:Class for custom mode. Empty leaves TTS
      UNAVAILABLE. Environment fallback: SPEECH_TTS_BACKEND_CLASS.
    default: ""
  # In custom mode a class string without ":" or a module that fails to
  # import fails Driver(CMD_INIT).
