#!/usr/bin/env python3
"""Scribe Mem search skill — CLI for history information retrieval.

Usage:
    python3 scripts/search_skill.py <query> [options]

Examples:
    # Semantic search
    python3 scripts/search_skill.py "where is the red cup"

    # Filtered search
    python3 scripts/search_skill.py "crafting tasks" --task build --success

    # Kitchen-only spatial search (Demo 1)
    python3 scripts/search_skill.py "cup" --scene kitchen

    # Task history search (Demo 2)
    python3 scripts/search_skill.py "fetch" --task fetch --success

    # Time-filtered search (last 1 hour)
    python3 scripts/search_skill.py "events" --last 3600

Options:
    --scene TYPE       Filter by scene_type (kitchen/living_room/workshop/outdoor)
    --action TYPE      Filter by action_type (grasp/place/navigate/craft/observe)
    --task TYPE        Filter by task_type (fetch/build/explore/dialogue)
    --success / --failure  Filter by outcome
    --difficulty MAX   Filter by max difficulty (easy/medium/hard)
    --top-k N          Number of results (default: 5)
    --alpha F          BM25 weight 0-1 (default: 0.3)
    --last SECONDS     Only results from the last N seconds
    --json             Output raw JSON instead of formatted text
    --data-dir DIR     Memory data directory (default: ~/.robonix/memory)
    --import-yaml PATH Import YAML data before searching
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import sys
import time as _time

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from memory_service.service import MemoryService
from memory_service.core.types import TagFilter, TimeRange

log = logging.getLogger("search_skill")


def build_tag_filter(args: argparse.Namespace) -> TagFilter:
    """Build TagFilter from CLI args."""
    tf = TagFilter()
    if args.scene:
        tf.scene_type = args.scene
    if args.action:
        tf.action_type = args.action
    if args.task:
        tf.task_type = args.task
    if args.success_flag and not args.failure_flag:
        tf.success = True
    elif args.failure_flag and not args.success_flag:
        tf.success = False
    if args.difficulty:
        tf.difficulty_max = args.difficulty
    return tf


async def main_async(args: argparse.Namespace) -> int:
    """Async main: init service, optionally import YAML, execute search."""
    data_dir = args.data_dir or os.path.join(os.path.expanduser("~"),
                                             ".robonix", "memory")
    svc = MemoryService(data_dir=data_dir)
    await svc.init()

    # Optional YAML import
    if args.import_yaml:
        from memory_service.core.builder import import_yaml_to_service
        yaml_path = os.path.expanduser(args.import_yaml)
        if os.path.exists(yaml_path):
            node_ids = await import_yaml_to_service(svc, yaml_path, clear_existing=True)
            print(f"[import] Loaded {len(node_ids)} records from {yaml_path}\n",
                  file=sys.stderr)
        else:
            print(f"[import] ERROR: YAML file not found: {yaml_path}",
                  file=sys.stderr)
            return 1

    # Build time range
    time_range = None
    if args.last:
        now = _time.time_ns()
        time_range = TimeRange(start_ts=now - int(args.last * 1_000_000_000))

    # Build tag filter
    tag_filter = build_tag_filter(args)
    if tag_filter.is_empty():
        tag_filter = None

    # Execute search
    resp = await svc.search(
        query=args.query,
        tags=tag_filter,
        top_k=args.top_k,
        alpha=args.alpha,
        time_range=time_range,
    )

    # Output
    if args.json:
        results = [n.to_dict() for n in resp.nodes]
        print(json.dumps({"count": len(results), "results": results},
                        indent=2, ensure_ascii=False))
    else:
        _format_output(resp.nodes, args)

    return 0


def _format_output(nodes, args: argparse.Namespace) -> None:
    """Pretty-print search results."""
    if not nodes:
        print("No memories found.")
        return

    print(f"\n{'='*70}")
    print(f"  Query: \"{args.query}\"  |  Results: {len(nodes)}")
    if args.scene:
        print(f"  Scene: {args.scene}", end="")
    if args.task:
        print(f"  |  Task: {args.task}", end="")
    if args.success_flag:
        print(f"  |  Success only", end="")
    print(f"\n{'='*70}\n")

    for i, n in enumerate(nodes):
        tags = n.tags
        scene = tags.scene_type if tags else "?"
        action = tags.action_type if tags else "?"
        task = tags.task_type if tags else "?"
        status = "✓" if (tags and tags.success) else "✗"
        ts_str = _ts_to_str(n.timestamp)

        print(f"  [{i+1}] {status} {n.summary}")
        print(f"      scene={scene}  action={action}  task={task}  "
              f"weight={n.weight:.2f}  node_id={n.node_id}")
        if n.spatial_data and n.spatial_data.objects:
            objs = ", ".join(f"{o.label} ({o.x:.1f}, {o.y:.1f}, {o.z:.1f})"
                           for o in n.spatial_data.objects[:3])
            print(f"      objects: {objs}")
        if n.causal_chain:
            print(f"      causal_chain: {n.causal_chain}")
        print(f"      time: {ts_str}")
        print()


def _ts_to_str(ts_ns: int) -> str:
    """Format nanosecond timestamp to human-readable."""
    import datetime
    dt = datetime.datetime.fromtimestamp(ts_ns / 1_000_000_000)
    return dt.strftime("%Y-%m-%d %H:%M:%S")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Scribe Mem search skill — history information retrieval",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("query", nargs="?", default="",
                        help="Search query text")
    parser.add_argument("--scene", help="Filter by scene_type")
    parser.add_argument("--action", help="Filter by action_type")
    parser.add_argument("--task", help="Filter by task_type")
    parser.add_argument("--success", dest="success_flag", action="store_true",
                        help="Only successful memories")
    parser.add_argument("--failure", dest="failure_flag", action="store_true",
                        help="Only failed memories (lessons)")
    parser.add_argument("--difficulty", choices=["easy", "medium", "hard"],
                        help="Max difficulty level")
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--alpha", type=float, default=0.3,
                        help="BM25 weight 0-1")
    parser.add_argument("--last", type=float,
                        help="Only memories from last N seconds")
    parser.add_argument("--json", action="store_true",
                        help="JSON output")
    parser.add_argument("--data-dir", help="Memory data directory")
    parser.add_argument("--import-yaml",
                        help="YAML file to import before search")
    parser.add_argument("--verbose", action="store_true")

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG, format="%(message)s")
    else:
        logging.disable(logging.CRITICAL)

    return asyncio.run(main_async(args))


if __name__ == "__main__":
    sys.exit(main())
