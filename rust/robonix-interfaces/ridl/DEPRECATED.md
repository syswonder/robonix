# RIDL syntax is deprecated

The `.ridl` custom syntax is no longer under active development. These files are kept for reference only.

## Alternatives

ROS IDL (`.msg` / `.srv` / `.action`) is the new canonical data definition format.

- Canonical sources: `robonix-interfaces/lib/robonix_msg/` (Robonix custom types) + `robonix-interfaces/lib/` (upstream ROS types)
- The `ridlc` tool targets ROS IDL → multi-transport code generation (e.g. protobuf, Rust, Python)

## References

- RFC001 deprecation notice: `docs/src/rfc/RFC001-RIDL.md`
