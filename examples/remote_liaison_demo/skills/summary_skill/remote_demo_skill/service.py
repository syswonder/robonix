from __future__ import annotations

import sys
from pathlib import Path

from robonix_api import Ok, Skill
from memory_mcp import Compact_Request, Compact_Response
from std_msgs_mcp import String


skill = Skill(id="summary_skill", namespace="robonix/skill/summary_skill")
notes_path = Path("../notes_skill/rbnx-build/data/demo_notes.md")


@skill.mcp("robonix/skill/summary_skill/compact")
async def compact(req: Compact_Request) -> Compact_Response:
    """Summarize the remote Liaison demo. Use this when the user asks for a recap, summary, or final demo status."""
    _ = req
    notes = "(no notes recorded yet)"
    if notes_path.exists():
        notes = notes_path.read_text(encoding="utf-8").strip() or notes
    text = (
        "Demo summary: macOS provides microphone and speaker audio over an SSH "
        "reverse tunnel, Liaison admits the speaker through the access gate, "
        "speech service performs ASR/TTS, Pilot plans a tool call, Executor "
        "dispatches the selected skill, and the result is spoken back. "
        f"Recorded notes:\n{notes}"
    )
    return Compact_Response(summary=String(data=text))


@skill.on_init
def init(cfg):
    """Accept boot config; this demo skill reads notes lazily."""
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
