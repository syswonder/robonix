# Robonix Speech Service (robonix/service/speech)
#
# Provides ASR (speech-to-text), TTS (text-to-speech), and Dialog (voice
# session) services over gRPC. Part of the srv (service) layer in the
# Robonix architecture, sitting above the primitive (primitive) audio driver.
#
# Key modules:
#   service.py   — gRPC servicers and main entry point
#   audio_utils  — audio format adaptation (any → 16kHz mono s16le)
#
# Entry point: python -m speech_service.service
