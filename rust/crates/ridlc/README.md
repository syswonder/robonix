# ridlc

Code generator: ROS IDL (`.msg` / `.srv` under `-I` paths) → **Python** / **Rust** / **Protobuf** (`--lang python` | `rust` | `proto`).

Custom `.ridl` syntax is deprecated; see `robonix-interfaces/ridl/DEPRECATED.md`.

```bash
cd rust
cargo run -p ridlc -- --help
```

## Related

- [robonix-interfaces](../../robonix-interfaces/README.md)
- [RFC001](../../../docs/src/rfc/RFC001-RIDL.md)
