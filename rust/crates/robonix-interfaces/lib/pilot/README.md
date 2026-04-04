# `pilot` (ROS IDL)

Wire types for **Liaison ↔ Pilot ↔ Executor** (`Intent`, `PilotEvent`, `TaskGraph`, …). `TaskGraph` is the BT-oriented task carrier; v1 encodes a linear list of `TaskCall` (see comment in `msg/TaskGraph.msg`).  
Per-package **`service PilotService`** is not generated. **`pilot.proto`** has **`.msg`** and contract-listed **`.srv`** messages; RPCs live in **`robonix_contracts.proto`**.
Contracts under `rust/contracts/sys/*.toml` reference these messages via `pilot/msg/...`.
