# Synchronization states mirror Chronos DomainState.
uint8 STATE_INITIALIZING=0
uint8 STATE_TRACKING=1
uint8 STATE_HOLDOVER=2
uint8 STATE_RESET=3
uint8 STATE_UNSYNCED=4
uint8 STATE_FAULT=5

# These fields describe current domain status.
string domain_id
uint64 generation
uint64 reference_generation
uint64 model_version
uint8 state
string reason

# model identity is carried separately because retained coefficients may be older.
# When has_model=false, consumers ignore model and encoders clear its payload.
bool has_model
chronos/msg/ClockModelParameters model
