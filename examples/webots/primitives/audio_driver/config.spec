# ALSA audio provider runtime config. Device fields may be omitted for
# automatic discovery. Explicit device names are recommended on robots with
# more than one USB audio interface.

config:
  mic_device: auto
  mic_sample_rate: probe
  mic_channels: 1
  mic_bits: 16
  mic_chunk_ms: 100
  speaker_device: auto
  speaker_sample_rate: 24000
  speaker_channels: 1
  speaker_bits: 16
