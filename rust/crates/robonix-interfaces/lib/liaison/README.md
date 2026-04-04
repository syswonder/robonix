# `liaison` (ROS IDL)

**`LiaisonService`** is **not** generated in `liaison.proto`; gRPC facades are in **`robonix_contracts.proto`**. `liaison.proto` contains **`.msg`** and contract-referenced **`_Request` / `_Response`** only.  
Stream payloads reuse `pilot/msg/PilotEvent.msg` as the sole **response** field on `HandleIntent.srv` when the contract uses `rpc_server_stream`.
