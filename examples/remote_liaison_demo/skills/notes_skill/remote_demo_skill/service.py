from __future__ import annotations

import sys
from datetime import datetime
from pathlib import Path

from robonix_api import Ok, Skill
from memory_mcp import Save_Request, Save_Response
from std_msgs_mcp import String


skill = Skill(id="notes_skill", namespace="robonix/skill/notes_skill")
notes_path = Path("rbnx-build/data/demo_notes.md")


@skill.mcp("robonix/skill/notes_skill/save")
async def save(req: Save_Request) -> Save_Response:
    """Record a remote demo note. Use this when the user asks you to remember, save, or record a demo fact."""
    content = req.content.data.strip()
    if not content:
        content = "(empty note)"
    notes_path.parent.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with notes_path.open("a", encoding="utf-8") as f:
        f.write(f"- {timestamp}: {content}\n")
    return Save_Response(
        confirmation=String(data=f"Recorded demo note to {notes_path}: {content}")
    )


@skill.on_init
def init(cfg):
    """Accept boot config and ensure the note directory exists."""
    _ = cfg
    notes_path.parent.mkdir(parents=True, exist_ok=True)
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
