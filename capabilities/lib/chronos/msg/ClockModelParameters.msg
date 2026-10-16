# Synchronization states mirror Chronos DomainState.
uint8 STATE_INITIALIZING=0
uint8 STATE_TRACKING=1
uint8 STATE_HOLDOVER=2
uint8 STATE_RESET=3
uint8 STATE_UNSYNCED=4
uint8 STATE_FAULT=5

# Identity belongs to these coefficients and may predate current domain status.
uint64 generation
uint64 reference_generation
uint64 model_version
uint8 state
int64 device_anchor_ns
int64 reference_anchor_ns
# Apply rate_adjust_ppb / 1e9 with nearest-integer, half-away-from-zero rounding.
int64 rate_adjust_ppb
int64 residual_offset_ns
uint64 uncertainty_ns
int64 valid_from_device_ns
int64 valid_until_device_ns
uint64 sample_count
uint64 last_observation_steady_ns
# When has_wrap_period=false, consumers ignore wrap_period_ns; encoders set it to zero.
bool has_wrap_period
uint64 wrap_period_ns
int64 latest_raw_device_ns
int64 wrap_offset_ns
