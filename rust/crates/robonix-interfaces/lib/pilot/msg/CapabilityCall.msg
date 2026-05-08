# One node in a Plan: invoke `contract_id` on `cap_id` with `args_json`.
# executor looks `cap_id` up in atlas at dispatch time, gets the endpoint,
# and routes via the cap's declared transport (currently MCP only for
# LLM-callable contracts).
string call_id
string cap_id
string contract_id
string args_json
