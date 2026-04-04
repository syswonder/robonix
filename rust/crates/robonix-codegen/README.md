# robonix-codegen

**ROS 2 IDL + contract codegen:** scans `-I` paths for `.msg` / `.srv`, optionally reads **`--contracts`** TOML, and writes generated outputs.

| `--lang` | Output |
|----------|--------|
| `proto` | One `*.proto` per ROS package → **Protobuf messages** (+ per-package `service` RPCs from `.srv`). With `--contracts`, also **`robonix_contracts.proto`** (`package robonix.contracts`): contract **`[mode].type`** is `rpc` \| `rpc_server_stream` \| `rpc_client_stream` \| `topic_out` \| `topic_in` (see `rust/contracts/README.md`). |
| `python` | `*_iox2.py` ctypes payloads for iceoryx2 Python (from `.msg` only). Does **not** emit contract services. |

Custom `.ridl` syntax is deprecated; see `crates/robonix-interfaces/ridl/DEPRECATED.md`.

```bash
cd rust
cargo run -p robonix-codegen -- --help
```

Typical proto regen (interfaces + contracts):

```bash
cargo run -p robonix-codegen -- --lang proto \
  -I crates/robonix-interfaces/lib \
  --contracts contracts \
  -o crates/robonix-interfaces/robonix_proto
```

## Related

- [Contracts (source of interface ids)](../../contracts/README.md)
- [robonix-interfaces](../robonix-interfaces/README.md)