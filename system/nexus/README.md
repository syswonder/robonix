# Nexus — communication libraries

One of the 12 Robonix system components, but not a process or a daemon.
Nexus is the set of transport libraries Robonix communicates over — the
gRPC, MCP, and ROS 2 client/server stacks that every component links
directly. There is nothing to start.

Robonix contracts are transport-agnostic; `rbnx codegen` projects each
onto gRPC / MCP / ROS 2, and those three stacks are nexus. atlas declares
per interface which transport a capability is reachable over, and
consumers dial it with the matching nexus library. The dev guide section
*能力与能力接口* covers that mechanism.

Roadmap: a self-built ROS 2 zero-copy / shared-memory transport for
high-rate sensor streams, added alongside the existing three.
