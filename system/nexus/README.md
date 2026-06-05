# Nexus — high-performance communication

One of the 12 Robonix system components. The transport-abstraction
layer that subsumes ROS 2, gRPC, MCP, and shared-memory transports
behind one capability-facing API.

**Status — v0.1 stub.** Not yet implemented.

Today every Robonix component picks its own transport directly: atlas
declares per-interface that a capability is reachable over gRPC / ROS
2 / MCP, and consumers dial that transport themselves. The dev guide
section *Capabilities and Capability Interfaces* covers the current mechanism.

When Nexus lands it will:

- own the transport-independent `(contract_id, transport) → endpoint`
  resolution that today is split between atlas and the Python /
  Rust client helpers,
- add zero-copy in-process / shared-memory transports for high-rate
  sensor streams,
- give a single back-pressure / cancellation model regardless of the
  underlying transport.
