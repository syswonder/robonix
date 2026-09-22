# One thing the reference could mean. Deliberately short: the point of this
# contract is that a caller no longer reads the whole registry.
string object_id
string cls
# What a person would call it: their correction if they made one.
string label
# Rank within this query only. Not comparable across queries.
float64 score
# Region it stands in, empty when it stands in none.
string region
# Metres from the robot, -1 when the robot pose is unknown.
float64 distance_m
# Seconds since last observed, -1 when unknown.
float64 last_seen_s
# True when it is remembered rather than currently seen. Scene keeps what it
# has seen, so it can answer about things out of view; saying which is the
# obligation that comes with that.
bool stale
# Which constraints this candidate satisfied, and which it failed. What makes
# a useful question possible: "both are in the living room, only one is by
# the sofa".
string[] matched
string[] missed
