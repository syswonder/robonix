# ridlc

**ROS 2 IDL + contract codegen:** scans `-I` paths for `.msg` / `.srv`, optionally reads **`--contracts`** TOML, and writes generated outputs.

| `--lang` | Output |
|----------|--------|
| `proto` | One `*.proto` per ROS package → **Protobuf messages** (+ per-package `service` RPCs from `.srv`). With `--contracts`, also **`robonix_contracts.proto`** (`package robonix.contracts`) mapping each contract id to gRPC `rpc` / streaming. |
| `python` | `*_iox2.py` ctypes payloads for iceoryx2 Python (from `.msg` only). Does **not** emit contract services. |

Custom `.ridl` syntax is deprecated; see `robonix-interfaces/ridl/DEPRECATED.md`.

```bash
cd rust
cargo run -p ridlc -- --help
```

Typical proto regen (interfaces + contracts):

```bash
cargo run -p ridlc -- --lang proto \
  -I robonix-interfaces/lib \
  --contracts contracts \
  -o robonix-interfaces/robonix_proto
```

## Related

- [Contracts (source of interface ids)](../../contracts/README.md)
- [robonix-interfaces](../../robonix-interfaces/README.md)
- [RFC001](../../../docs/src/rfc/RFC001-RIDL.md)
