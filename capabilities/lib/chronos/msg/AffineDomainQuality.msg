# Synchronization states mirror Chronos DomainState.
uint8 STATE_INITIALIZING=0
uint8 STATE_TRACKING=1
uint8 STATE_HOLDOVER=2
uint8 STATE_RESET=3
uint8 STATE_UNSYNCED=4
uint8 STATE_FAULT=5

string domain_id
uint64 generation
uint64 reference_generation
uint64 model_version
uint8 state
uint64 accepted_observations
uint64 rejected_observations
# When a has_* field is false, consumers ignore and encoders clear its payload.
bool has_last_observation_age
uint64 last_observation_age_ns
bool has_uncertainty
uint64 uncertainty_ns
string reason
