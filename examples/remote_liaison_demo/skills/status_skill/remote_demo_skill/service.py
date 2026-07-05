from __future__ import annotations

import sys
from datetime import datetime

from robonix_api import Ok, Skill
from memory_mcp import Search_Request, Search_Response
from std_msgs_mcp import String


skill = Skill(id="status_skill", namespace="robonix/skill/status_skill")


@skill.mcp("robonix/skill/status_skill/search")
async def search(req: Search_Request) -> Search_Response:
    """Query the remote interaction demo status. Use this when the user asks about system status, remote connection state, or what is currently running."""
    query = req.query.data.strip()
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    text = (
        f"Remote Liaison demo is running at {now}. "
        "Active path: macOS audio bridge -> access gate -> speech ASR -> "
        "Liaison -> Pilot -> Executor -> MCP skill -> speech TTS. "
        "This demo intentionally does not start Webots, camera, mapping, or navigation. "
        f"User query was: {query or '(empty)'}"
    )
    return Search_Response(results=String(data=text))


@skill.on_init
def init(cfg):
    """Accept boot config; this demo skill has no external resources."""
    _ = cfg
    return Ok()


@skill.on_activate
def activate():
    """Mark the skill ready when Executor activates it for a tool call."""
    return Ok()


def main() -> int:
    skill.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
