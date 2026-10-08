# The `system.liaison:` block of robonix_manifest.yaml. rbnx boot passes the
# whole block to robonix-liaison as --config-json and also turns listen, atlas,
# pilot_endpoint and log into CLI flags. The hands-free keys are read from the
# JSON; a value of the wrong type stops Liaison at startup ("parse liaison
# hands-free config"). Other keys are ignored.
#
# Access control (ROBONIX_LIAISON_ACCESS_ENABLED, ROBONIX_LIAISON_ALLOWED_USERS,
# ROBONIX_LIAISON_VOICE_THRESHOLD) is read from the environment only.

specVersion: 1
description: >-
  Configuration of Liaison, the user-facing entry that turns text and voice
  requests into Pilot tasks and can listen for a wake word on the robot.

properties:
  listen:
    type: string
    default: 0.0.0.0:50081
    description: >-
      gRPC listen address, host:port. Liaison itself also accepts a bare port
      and binds 0.0.0.0 on it, but rbnx boot checks the address before spawning
      and rejects a bare port. When unset, the port comes from
      ROBONIX_LIAISON_PORT, else 50081, on 0.0.0.0.

  atlas:
    type: string
    description: >-
      Atlas endpoint. When unset, rbnx passes system.atlas.listen. Without
      either, Liaison uses ROBONIX_ATLAS_ENDPOINT, then ROBONIX_ATLAS, then
      127.0.0.1:50051.

  pilot_endpoint:
    type: string
    default: 127.0.0.1:50071
    description: >-
      Pilot endpoint used when Atlas cannot resolve Pilot. Environment
      fallback: ROBONIX_PILOT_ENDPOINT. `localhost` is rewritten to 127.0.0.1.

  log:
    type: string
    enum: [debug, info, warn, warning, error]
    default: info
    description: >-
      Lowest level written to Liaison's log file. Matched case-insensitively;
      an unrecognised value is ignored. When unset, SCRIBE_FILE_LEVEL applies,
      then info.

  handsfree_enabled:
    type: boolean
    default: false
    x-group: Hands-free voice
    description: >-
      Listen for a wake word at startup. Takes effect only when both
      handsfree_mic_provider_id and handsfree_speaker_provider_id are set;
      otherwise hands-free mode starts disabled. A client can switch it on
      later and choose the providers at that point.

  handsfree_mic_provider_id:
    type: string
    x-provider: robonix/primitive/audio/mic
    default: ""
    x-group: Hands-free voice
    description: >-
      Provider id or namespace of the robonix/primitive/audio/mic provider to
      capture from. A named provider must be registered in Atlas; Liaison does
      not fall back to another one.

  handsfree_speaker_provider_id:
    type: string
    x-provider: robonix/primitive/audio/speaker
    default: ""
    x-group: Hands-free voice
    description: >-
      Provider id or namespace of the audio output that plays the
      acknowledgement and the spoken reply.

  handsfree_speech_provider_id:
    type: string
    x-provider: robonix/service/speech/asr_stream
    default: speech
    x-group: Hands-free voice
    description: >-
      Speech provider used for wake-word detection, speech recognition and
      text-to-speech. An empty string accepts any provider of the contract.

  handsfree_voiceprint_provider_id:
    type: string
    x-provider: robonix/service/voiceprint/identify
    default: voiceprint
    x-group: Hands-free voice
    description: >-
      Voiceprint provider that identifies the speaker before the request
      reaches Pilot. An empty string accepts any provider of the contract.

  handsfree_ack_text:
    type: string
    default: "\u6211\u5728"  # Mandarin "I'm here"
    x-group: Hands-free voice
    description: >-
      Spoken after the wake word, before recording starts. Blank skips it. It
      is also skipped when the wake word interrupts a turn already running.

  handsfree_session_id:
    type: string
    default: handsfree
    x-group: Hands-free voice
    description: Session id that hands-free turns are submitted under.

  handsfree_record_seconds:
    type: integer
    minimum: 0
    maximum: 4294967295
    default: 20
    x-group: Hands-free voice
    description: >-
      Upper bound on one recording, in seconds. Speech detection usually ends
      the turn earlier. 0 means 10; values below 5 are raised to 5.
