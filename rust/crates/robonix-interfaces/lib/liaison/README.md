# `liaison` (ROS IDL)

**`LiaisonService`** RPCs: `lib/liaison/srv/*.srv` → ridlc → `robonix_proto/liaison.proto`.  
Stream payloads reuse `pilot/msg/PilotEvent.msg` as the sole **response** field on `HandleIntent.srv` when the contract uses `rpc_server_stream`.
