# Runtime configuration accepted by the Linux ALSA audio primitive.
#
# This documents the mapping passed as the package instance's `config:` value.
# It is not loaded as a schema. Values below are runtime defaults.

config:
  # string ALSA device id or null, default: null.
  # Examples: hw:1,0 or plughw:1,0. Null selects the detected default input.
  mic_device: null

  # integer hertz or null, default: null.
  # Null probes the selected microphone and prefers 16000 Hz when supported.
  mic_sample_rate: null

  # integer channel count, default: 1; must be supported by the device.
  # ASR input is normally mono.
  mic_channels: 1

  # integer bits per sample, default: 16; accepted: 8, 16, 24, 32.
  # Selects the raw signed PCM format passed to arecord.
  mic_bits: 16

  # integer milliseconds, default: 100; must be greater than zero.
  # Duration of each microphone chunk returned by the streaming capability.
  mic_chunk_ms: 100

  # string ALSA device id or null, default: null.
  # Examples: hw:1,0 or plughw:1,0. Null selects the detected default output.
  speaker_device: null

  # integer hertz, default: 24000; must be supported by the output device.
  # It must match the PCM rate produced by the configured TTS path.
  speaker_sample_rate: 24000

  # integer channel count, default: 1; must be supported by the device.
  # Robonix TTS output is normally mono.
  speaker_channels: 1

  # integer bits per sample, default: 16; accepted: 8, 16, 24, 32.
  # Selects the raw signed PCM format passed to aplay.
  speaker_bits: 16
