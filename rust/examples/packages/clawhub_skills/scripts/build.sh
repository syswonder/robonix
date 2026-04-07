#!/usr/bin/env bash
# build.sh — Pull selected skills from OpenClaw ClawHub (openclaw/skills repo)
# and prepare them for Robonix import.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
SKILLS_DIR="$PKG/skills"
BUILD_DIR="$PKG/rbnx-build"
SKILLS_REPO="https://github.com/openclaw/skills.git"
SKILLS_CACHE="$PKG/.clawhub_cache"

if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
    rm -rf "$BUILD_DIR" "$SKILLS_DIR" "$SKILLS_CACHE"
fi

echo "[clawhub_skills] build started"

# ── 1. Clone or update the openclaw/skills repo ─────────────────────────────
if [ -d "$SKILLS_CACHE/.git" ]; then
    echo "[clawhub_skills] updating skills cache..."
    (cd "$SKILLS_CACHE" && git pull --ff-only 2>/dev/null) || true
else
    echo "[clawhub_skills] cloning openclaw/skills..."
    git clone --depth 1 "$SKILLS_REPO" "$SKILLS_CACHE"
fi

# ── 2. Select and import skills ─────────────────────────────────────────────
# Pick a curated set of representative skills that are useful for robotics
# and general agent tasks. Each skill is a directory with SKILL.md.
SELECTED_SKILLS=(
    # Agent reasoning & planning
    "task-planner"          # Task management, priorities, deadlines
    "deep-research"         # Multi-source web research with citations
    "memory"                # Persistent agent memory system
    # Developer / DevOps
    "code-review"           # Systematic code review patterns
    "commit"                # Git commit with contextual message
    # System operations
    "system-monitor"        # CPU/RAM/GPU monitoring
    "web-search"            # Web search integration
)

mkdir -p "$SKILLS_DIR"

imported=0
for skill_name in "${SELECTED_SKILLS[@]}"; do
    # Skills in openclaw/skills are organized as: skills/<author>/<skill>/SKILL.md
    # or sometimes directly: skills/<skill>/SKILL.md
    # Search for the skill by name
    skill_src=$(find "$SKILLS_CACHE" -type f -name "SKILL.md" -path "*/${skill_name}/*" | head -1)

    if [ -z "$skill_src" ]; then
        echo "[clawhub_skills] warning: skill '${skill_name}' not found in repo, skipping"
        continue
    fi

    skill_dir=$(dirname "$skill_src")
    target_dir="$SKILLS_DIR/$skill_name"

    # Copy the skill directory (SKILL.md + any references/)
    rm -rf "$target_dir"
    cp -r "$skill_dir" "$target_dir"

    echo "[clawhub_skills] imported: $skill_name"
    imported=$((imported + 1))
done

echo "[clawhub_skills] imported $imported skills from ClawHub"

# ── 3. Stamp ─────────────────────────────────────────────────────────────────
mkdir -p "$BUILD_DIR"
date -Iseconds > "$BUILD_DIR/.rbnx-built"
echo "[clawhub_skills] build complete"
