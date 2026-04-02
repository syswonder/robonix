# `pilot` (ROS IDL)

Wire types for **Liaison ↔ Pilot ↔ Executor** (`Intent`, `PilotEvent`, `TaskGraph`, …). `TaskGraph` is the BT-oriented task carrier; v1 encodes a linear list of `TaskCall` (see comment in `msg/TaskGraph.msg`).  
`PilotService` RPCs come from `lib/pilot/srv/*.srv` → ridlc → `robonix_proto/pilot.proto`.
Contracts under `rust/contracts/sys/*.toml` reference these messages via `pilot/msg/...`.
