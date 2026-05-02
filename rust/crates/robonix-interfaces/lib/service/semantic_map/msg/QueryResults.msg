# Wrapper around `Object[] results` so codegen can emit a single-typed
# response for scene/query (robonix-py's @mcp_contract requires the
# handler return a single codegen message, not a bare array).
Object[] results
