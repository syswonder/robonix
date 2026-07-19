uint32 schema_version
string body_id
uint64 seq
int64 source_ts_ns
int64 soma_ts_ns
uint32 ttl_ms

ComponentStatus[] components
ActuatorState[] actuators
PowerSourceState[] power_sources
SafetyState safety
SafetyEndpointState[] safety_endpoints
FaultState[] faults
Metric[] metrics
