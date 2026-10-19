# Refresh health is separate from clock evidence so initial failure is reportable.
# When a has_* field is false, consumers ignore and encoders clear its payload.
bool has_last_successful_refresh_steady
uint64 last_successful_refresh_steady_ns
bool has_failure
chronos/msg/RefreshFailure failure
