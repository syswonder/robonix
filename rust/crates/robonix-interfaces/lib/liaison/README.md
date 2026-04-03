# `liaison` (ROS IDL)

**`LiaisonService`** RPCs: `lib/liaison/srv/*.srv` → ridlc → `robonix_proto/liaison.proto`.  
Stream payloads reuse `pilot/msg/PilotEvent.msg` via `@robonix.grpc stream_server` on `HandleIntent.srv`.
