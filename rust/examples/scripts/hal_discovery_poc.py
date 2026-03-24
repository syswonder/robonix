#!/usr/bin/env python3
"""Namespace discovery PoC: list nodes/interfaces from the single global robonix-server (no ROS required).

Run:  ROBONIX_SERVER=127.0.0.1:50051 python3 hal_discovery_poc.py

Optional env (legacy HAL_* names still accepted):
  ABSTRACT_INTERFACE_ID  Exact match on InterfaceInfo.abstract_interface_id (ignores NAMESPACE_PREFIX/INTERFACE_NAME when set)
  NAMESPACE_PREFIX       Prefix filter for QueryNodes (e.g. robonix/prm, robonix/sys)
  INTERFACE_NAME         Filter by DeclareInterface.name
  TRANSPORT              Filter by supported transport
  DISTRO_PREFIX          Filter by node.distro prefix
  CONTAINER_ID           Filter by exact container_id
  GROUP_BY_DOMAIN        If "1", print a short summary line per node under prm vs sys
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

_RUST = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_RUST / "examples" / "proto_gen"))

import grpc  # noqa: E402

import robonix_runtime_pb2 as pb  # noqa: E402
import robonix_runtime_pb2_grpc as pb_grpc  # noqa: E402


def _env(primary: str, legacy: str) -> str:
    return os.environ.get(primary, os.environ.get(legacy, ""))


def _target() -> str:
    s = os.environ.get("ROBONIX_SERVER", "127.0.0.1:50051")
    for prefix in ("http://", "https://"):
        if s.startswith(prefix):
            s = s[len(prefix) :]
    return s


def main() -> int:
    abstract = os.environ.get("ABSTRACT_INTERFACE_ID", "").strip()
    ns = _env("NAMESPACE_PREFIX", "HAL_NAMESPACE_PREFIX")
    name = _env("INTERFACE_NAME", "HAL_INTERFACE_NAME")
    transport = _env("TRANSPORT", "HAL_TRANSPORT")

    ch = grpc.insecure_channel(_target())
    stub = pb_grpc.RobonixRuntimeStub(ch)
    resp = stub.QueryNodes(
        pb.QueryNodesRequest(
            namespace="" if abstract else ns,
            name="" if abstract else name,
            transport=transport,
            distro_prefix=_env("DISTRO_PREFIX", "HAL_DISTRO_PREFIX"),
            container_id=_env("CONTAINER_ID", "HAL_CONTAINER_ID"),
            abstract_interface_id=abstract,
        )
    )

    print("[hal_poc] QueryNodes filters:")
    if abstract:
        print(f"  abstract_interface_id={abstract!r} transport={transport!r}")
    else:
        print(f"  namespace_prefix={ns!r} name={name!r} transport={transport!r}")
    print(f"[hal_poc] {len(resp.nodes)} node(s)\n")

    group = _env("GROUP_BY_DOMAIN", "HAL_GROUP_BY_DOMAIN").strip() in ("1", "true", "yes")
    if group and not ns and not abstract:
        prm = sum(1 for n in resp.nodes if n.namespace.startswith("robonix/prm"))
        sysn = sum(1 for n in resp.nodes if n.namespace.startswith("robonix/sys"))
        other = len(resp.nodes) - prm - sysn
        print(f"[hal_poc] by prefix: robonix/prm={prm}  robonix/sys={sysn}  other={other}\n")

    for n in resp.nodes:
        print(f"  node_id={n.node_id!r}")
        print(f"  namespace (mount)={n.namespace!r}")
        print(f"  kind={n.kind!r} distro={n.distro!r} container_id={n.container_id!r}")
        for iface in n.interfaces:
            print(
                f"    interface name={iface.name!r} abstract_interface_id={iface.abstract_interface_id!r} transports={list(iface.supported_transports)}"
            )
            if iface.metadata_json:
                try:
                    meta = json.loads(iface.metadata_json)
                    ep = meta.get("endpoint", "")
                    tools = meta.get("tools")
                    if ep:
                        print(f"      allocated data-plane endpoint: {ep!r}")
                    if isinstance(tools, list) and tools:
                        names = [t.get("name", "?") for t in tools if isinstance(t, dict)]
                        print(f"      tools (concrete ops): {names[:12]}{'...' if len(names) > 12 else ''}")
                except json.JSONDecodeError:
                    print(f"      metadata_json (raw): {iface.metadata_json[:120]}...")
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
