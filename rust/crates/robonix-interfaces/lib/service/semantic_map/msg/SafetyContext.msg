# Stub returned by scene/get_safety_context. Sentinel will eventually
# populate this with real forbidden zones + per-tracked-human distances.
# v1 returns status="not_implemented" plus empty arrays so Pilot can
# still call the tool without exception.
string status                       # "ok" | "not_implemented" | "error"
string[] forbidden_zones            # opaque tags or polygon ids; empty for now
string human_distances_json         # JSON object {<human_id>: <metres>}; empty for now
