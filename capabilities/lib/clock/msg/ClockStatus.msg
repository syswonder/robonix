uint32 STATE_UNSPECIFIED=0
uint32 STATE_LOCKED=1
uint32 STATE_HOLDOVER=2
uint32 STATE_UNSYNCED=3
uint32 STATE_FAULT=4

string domain_id
uint64 generation
uint64 sequence_id
uint32 state
string grandmaster_id
uint32 observed_domain_number
int64 master_offset_ns
uint64 mean_path_delay_ns
uint64 uncertainty_ns
uint64 sample_age_ns
string source
string reason
