# state: 0=PENDING 1=RUNNING 2=SUCCEEDED 3=FAILED 4=CANCELED 5=TIMEOUT 6=PAUSED
string call_id
string provider_id
string contract_id
string run_id
uint32 state
string detail
# Terminal states carry the merged capability call outcome.
bool success
string output
string error
