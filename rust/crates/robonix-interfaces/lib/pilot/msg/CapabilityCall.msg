# One node in a Plan: invoke `contract_id` on `provider_id` with `args_json`.
# executor looks `provider_id` up in atlas at dispatch time, gets the endpoint,
# and routes via the cap's declared transport (currently MCP only for
# LLM-callable contracts).
string call_id
string provider_id
string contract_id
string args_json
