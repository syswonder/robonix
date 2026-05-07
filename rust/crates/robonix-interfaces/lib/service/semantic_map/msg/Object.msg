# One tracked thing in the world. Six fields, all primitives — no
# nested types, no enums. Position is map-frame xyz; the registry
# only emits objects it currently believes exist (no missing flag,
# no observation_count: that's internal telemetry, not API).

string id
string label
float64 x
float64 y
float64 z
float64 last_seen_unix
