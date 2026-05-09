# Robonix Audio Driver (robonix/primitive/audio)
#
# Primitive-layer audio driver for microphone capture and speaker playback.
# Auto-discovers ALSA devices and provides gRPC streaming interfaces for
# both input (mic) and output (speaker).
#
# This package is the REFERENCE TEMPLATE for all future primitive driver packages.
#
# Key modules:
#   node.py          — Main entry point (Atlas registration + gRPC servers)
#   alsa_utils.py    — ALSA device scanning + driver registry
#   mic_driver.py    — Microphone capture via arecord subprocess
#   speaker_driver.py — Speaker playback via aplay subprocess
#
# Entry point: python -m audio_driver.main
