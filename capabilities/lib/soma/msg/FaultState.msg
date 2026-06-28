uint8 INFO=0
uint8 WARN=1
uint8 ERROR=2
uint8 CRITICAL=3

string component_id
string fault_id
uint8 severity
bool active
bool clearable
int64 onset_ts_ns

uint32 vendor_code
string vendor_code_text
string message
string[] attributes
string vendor_raw_json
