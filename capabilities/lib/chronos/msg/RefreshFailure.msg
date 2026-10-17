# One continuing Chronos quality-refresh failure.
# When a has_* field is false, consumers ignore and encoders clear its payload.
string message
uint64 consecutive_failures
bool has_first_observed_steady
uint64 first_observed_steady_ns
bool has_last_observed_steady
uint64 last_observed_steady_ns
