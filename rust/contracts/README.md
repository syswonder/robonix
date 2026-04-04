# Robonix contracts (`rust/contracts`)

**`[contract].id`** is the canonical logical interface id. Each `*.toml` sets **`[contract]`**, exactly one **`[io.*]`** table, and **`[mode].type`**, which fixes how **`robonix_contracts.proto`** emits the gRPC facade for that id.

ROS 2 IDL lives under `crates/robonix-interfaces/lib/**` (`.msg` / `.srv`). Legacy **`# @robonix.grpc`** lines at the top of `.srv` files are ignored by codegen. **Streaming** is wired only by **`[mode].type`** plus the **shape of the `.srv`**:

| `[mode].type` | Stream element in `.srv` |
|---------------|---------------------------|
| **`rpc_server_stream`** | **Response** section must contain **exactly one** field — its type is each item in the server stream (a named `pkg/msg`, not an array). |
| **`rpc_client_stream`** | **Request** section must contain **exactly one** field — its type is each client stream element. Unary **response** is still the `.srv` response (empty → `google.protobuf.Empty`). |

**`[io.srv]`** sets **`srv = "pkg/srv/Name"`**; **`[io.msg]`** sets **`msg = "pkg/msg/Name"`** (exactly one of the two `[io.*]` tables).

---

## `[mode].type` → `robonix_contracts.proto`

| `[mode].type` | `[io]` |
|---------------|--------|
| **`rpc`** | **`[io.srv]`** — unary `Call(Request) returns (Response)`. |
| **`rpc_server_stream`** | **`[io.srv]`** — `Stream(Request…) returns (stream Elem)`; Elem from **response** single field. |
| **`rpc_client_stream`** | **`[io.srv]`** — `Stream(stream Elem) returns (Response…)`; Elem from **request** single field. |
| **`topic_out`** / **`topic_in`** | **`[io.msg]`** — `msg = "pkg/msg/Name"` |

---

## Example

**`rpc_server_stream`** — `Execute.srv`:

```text
pilot/TaskGraph graph
---
executor/TaskCallEvent event
```

**`rpc_client_stream`** — `StreamMove.srv`:

```text
MoveCommand cmd
---
```

```toml
[io.srv]
srv = "prm_base/srv/StreamMove"

[mode]
type = "rpc_client_stream"
```

---

## Regenerate

From `rust/`:

```bash
cargo run -p robonix-codegen -- --lang proto \
  -I crates/robonix-interfaces/lib \
  --contracts contracts \
  -o crates/robonix-interfaces/robonix_proto
```

Do not hand-edit generated `robonix_proto/*.proto`.
