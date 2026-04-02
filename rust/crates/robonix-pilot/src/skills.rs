//! OpenClaw-style skills: optional `SOUL.md`, extra skill roots, precedence merge, and a
//! compact skill index for the system prompt. See `https://docs.openclaw.ai/skills/`.

use anyhow::{Context, Result};
use robonix_sdk::{RobonixClient, SkillEntry};
use serde::Deserialize;
use serde_json::{Value, json};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// One skill visible to the agent after merging registry + local skill folders.
#[derive(Clone, Debug)]
pub struct AgentSkill {
    pub name: String,
    pub description: String,
    pub path: String,
    pub node_id: String,
    pub metadata_json: String,
}

/// Optional agent personality / operating instructions (OpenClaw `SOUL.md`).
/// Resolution: `ROBONIX_PILOT_SOUL` path, else `~/.robonix/SOUL.md`.
pub fn load_agent_soul() -> Option<String> {
    if let Ok(p) = std::env::var("ROBONIX_PILOT_SOUL") {
        let p = p.trim();
        if !p.is_empty() {
            return std::fs::read_to_string(p).ok();
        }
    }
    let home = std::env::var_os("HOME").map(PathBuf::from)?;
    let soul = home.join(".robonix").join("SOUL.md");
    if soul.is_file() {
        return std::fs::read_to_string(soul).ok();
    }
    None
}

#[derive(Deserialize)]
struct FrontMatter {
    name: String,
    description: String,
    #[serde(default, rename = "disable-model-invocation")]
    disable_model_invocation: Option<bool>,
    #[serde(default, rename = "user-invocable")]
    user_invocable: Option<bool>,
}

fn parse_skill_md(path: &Path) -> Result<(FrontMatter, String)> {
    let content =
        std::fs::read_to_string(path).with_context(|| format!("read {}", path.display()))?;
    let trimmed = content.trim_start();
    if !trimmed.starts_with("---") {
        anyhow::bail!("SKILL.md must start with YAML frontmatter (---)");
    }
    let after_first = &trimmed[3..];
    let end = after_first
        .find("\n---")
        .ok_or_else(|| anyhow::anyhow!("SKILL.md: missing closing --- for frontmatter"))?;
    let yaml_block = &after_first[..end];
    let fm: FrontMatter = serde_yaml::from_str(yaml_block)
        .with_context(|| format!("invalid YAML frontmatter in {}", path.display()))?;
    if fm.name.trim().is_empty() {
        anyhow::bail!("SKILL.md: name must not be empty");
    }
    let disable = fm.disable_model_invocation.unwrap_or(false);
    let user_invocable = fm.user_invocable.unwrap_or(true);
    let metadata_json = json!({
        "disable_model_invocation": disable,
        "user_invocable": user_invocable,
    })
    .to_string();
    Ok((fm, metadata_json))
}

fn skill_body_after_frontmatter(content: &str) -> &str {
    let trimmed = content.trim_start();
    if !trimmed.starts_with("---") {
        return trimmed;
    }
    let after_first = &trimmed[3..];
    if let Some(end) = after_first.find("\n---") {
        after_first[end + 4..].trim_start()
    } else {
        ""
    }
}

/// Scan `<root>/skills/<name>/SKILL.md` (package layout) or, if `root` is already a `skills` tree,
/// `<root>/<name>/SKILL.md`.
fn scan_skills_tree(skills_root: &Path, node_id: &str) -> Vec<AgentSkill> {
    let mut out = Vec::new();
    let entries = match std::fs::read_dir(skills_root) {
        Ok(e) => e,
        Err(_) => return out,
    };
    for entry in entries.flatten() {
        if !entry.file_type().map(|t| t.is_dir()).unwrap_or(false) {
            continue;
        }
        let skill_md = entry.path().join("SKILL.md");
        if !skill_md.is_file() {
            continue;
        }
        if let Ok((fm, metadata_json)) = parse_skill_md(&skill_md) {
            let path = skill_md.canonicalize().unwrap_or(skill_md);
            out.push(AgentSkill {
                name: fm.name,
                description: fm.description,
                path: path.display().to_string(),
                node_id: node_id.to_string(),
                metadata_json,
            });
        }
    }
    out.sort_by(|a, b| a.name.cmp(&b.name));
    out
}

fn scan_package_skills_dir(package_root: &Path, node_id: &str) -> Vec<AgentSkill> {
    let skills_dir = package_root.join("skills");
    scan_skills_tree(&skills_dir, node_id)
}

/// Extra skill locations (OpenClaw-style): `ROBONIX_SKILLS_EXTRA_DIRS` — `:`-separated absolute
/// or relative paths; each may be a package root (with `skills/`) or a directory that directly
/// contains skill subfolders (`<dir>/<skill>/SKILL.md`).
pub fn scan_extra_skill_dirs() -> Vec<AgentSkill> {
    let Ok(raw) = std::env::var("ROBONIX_SKILLS_EXTRA_DIRS") else {
        return Vec::new();
    };
    let node_id = "com.robonix.local.workspace".to_string();
    let mut merged: HashMap<String, AgentSkill> = HashMap::new();
    for part in raw.split(|c| c == ':' || c == ';') {
        let p = part.trim();
        if p.is_empty() {
            continue;
        }
        let path = PathBuf::from(p);
        let mut batch = scan_package_skills_dir(&path, &node_id);
        if batch.is_empty() {
            batch = scan_skills_tree(&path, &node_id);
        }
        for s in batch {
            merged.insert(s.name.clone(), s);
        }
    }
    let mut v: Vec<_> = merged.into_values().collect();
    v.sort_by(|a, b| a.name.cmp(&b.name));
    v
}

/// Managed skills: `~/.robonix/skills/<name>/SKILL.md`
pub fn scan_home_skills() -> Vec<AgentSkill> {
    let Some(home) = std::env::var_os("HOME").map(PathBuf::from) else {
        return Vec::new();
    };
    let root = home.join(".robonix").join("skills");
    scan_skills_tree(&root, "com.robonix.local.managed")
}

fn registry_to_skills(entries: Vec<SkillEntry>) -> Vec<AgentSkill> {
    let mut out = Vec::new();
    for e in entries {
        for s in e.skills {
            out.push(AgentSkill {
                name: s.name,
                description: s.description,
                path: s.path,
                node_id: e.node_id.clone(),
                metadata_json: s.metadata_json,
            });
        }
    }
    out.sort_by(|a, b| a.name.cmp(&b.name));
    out
}

/// Merge order (OpenClaw-like): **registry** (lowest) → **~/.robonix/skills** → **`ROBONIX_SKILLS_EXTRA_DIRS`** (last path wins on name clash).
pub async fn load_merged_skills(sdk: &mut RobonixClient) -> Result<Vec<AgentSkill>> {
    let mut by_name: HashMap<String, AgentSkill> = HashMap::new();
    let reg = sdk.query_all_skills().await?;
    for s in registry_to_skills(reg) {
        by_name.entry(s.name.clone()).or_insert(s);
    }
    for s in scan_home_skills() {
        by_name.insert(s.name.clone(), s);
    }
    for s in scan_extra_skill_dirs() {
        by_name.insert(s.name.clone(), s);
    }
    let mut v: Vec<_> = by_name.into_values().collect();
    v.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(v)
}

fn xml_escape(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}

pub fn skill_disable_model_invocation(metadata_json: &str) -> bool {
    let Ok(v): Result<Value, _> = serde_json::from_str(metadata_json) else {
        return false;
    };
    v.get("disable_model_invocation")
        .and_then(|x| x.as_bool())
        .unwrap_or(false)
}

/// Skills eligible for model prompt (not `disable_model_invocation`).
pub fn model_visible_skills(skills: &[AgentSkill]) -> Vec<&AgentSkill> {
    skills
        .iter()
        .filter(|s| !skill_disable_model_invocation(&s.metadata_json))
        .collect()
}

/// Compact XML index (OpenClaw / pi-style list for deterministic layout).
pub fn format_skills_xml(skills: &[&AgentSkill]) -> String {
    if skills.is_empty() {
        return String::new();
    }
    let mut lines = vec!["<skills>".to_string()];
    for s in skills {
        lines.push(format!(
            "  <skill name=\"{}\" description=\"{}\" path=\"{}\" node=\"{}\"/>",
            xml_escape(&s.name),
            xml_escape(&s.description),
            xml_escape(&s.path),
            xml_escape(&s.node_id),
        ));
    }
    lines.push("</skills>".to_string());
    lines.join("\n")
}

/// Optional injection of markdown bodies (after frontmatter), capped for context size.
pub fn format_skill_playbooks(skills: &[&AgentSkill]) -> String {
    let max_total: usize = std::env::var("ROBONIX_SKILLS_INJECT_MAX_CHARS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(16_000);
    let per_skill: usize = std::env::var("ROBONIX_SKILLS_INJECT_PER_SKILL_CHARS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(4_000);

    if max_total == 0 {
        return String::new();
    }

    let mut budget = max_total;
    let mut blocks = Vec::new();
    for s in skills {
        if budget == 0 {
            break;
        }
        let Ok(text) = std::fs::read_to_string(&s.path) else {
            continue;
        };
        let body = skill_body_after_frontmatter(&text);
        let mut chunk = if body.len() > per_skill {
            format!(
                "### {} (truncated)\n{}\n\n… (full instructions: read_file `{}`)\n",
                s.name,
                &body[..per_skill.min(body.len())],
                s.path
            )
        } else {
            format!("### {}\n{}\n", s.name, body)
        };
        if chunk.len() > budget {
            chunk.truncate(budget);
            chunk.push_str("\n… (budget exhausted)\n");
        }
        budget = budget.saturating_sub(chunk.len());
        blocks.push(chunk);
    }
    if blocks.is_empty() {
        return String::new();
    }
    format!(
        "## Skill playbooks (injected excerpts)\n\n\
         When a section below is **not** marked truncated, you may treat it as sufficient \
         to **Execute** without an extra `read_file`. If marked truncated or missing, \
         call **read_file** on that skill's `path` from the index before using HAL/MCP tools.\n\n{}",
        blocks.join("\n---\n\n")
    )
}

// ── System prompt builder ─────────────────────────────────────────────────────

/// Build the VLM system prompt for a Pilot turn.
///
/// `tool_summary` is a short text block listing available tools (injected by
/// Planner after fetching from Executor).  Skills are loaded from Atlas and
/// local directories via `load_merged_skills`.
pub fn build_system_prompt(soul: Option<&str>, skills: &[AgentSkill]) -> String {
    let mut p = String::new();

    if let Some(s) = soul {
        let t = s.trim();
        if !t.is_empty() {
            p.push_str("## Agent SOUL\n\n");
            p.push_str(t);
            p.push_str("\n\n---\n\n");
        }
    }

    p.push_str(
        "\
You are the Robonix Pilot — the reasoning and planning component of a robot system.
You receive requests from a user or higher-level system and translate them into actions
by calling the tools available to you.

## Operating principles
- ACT immediately using your tools. Do not ask the user to run things themselves.
- Execute all necessary steps before replying.
- Each tool call you make is dispatched to the Executor runtime, which handles the
  actual robot hardware or service call.
- When a skill playbook is available (see Skills index below), READ it with read_file
  before executing — it contains exact step sequences, forbidden operations, and
  safety limits.
- Prefer structured output; report tool results concisely.
- If a tool returns an error, diagnose and retry or report to the user.

## Skill workflow
1. **Select** — pick the skill(s) whose name/description match the goal.
2. **Read**   — call read_file on the skill's `path` to load the playbook.
3. **Execute** — follow the playbook steps using the available tools.

",
    );

    // Skills index.
    let visible = model_visible_skills(skills);
    if !visible.is_empty() {
        p.push_str("## Skills index\n\n");
        p.push_str(&format_skills_xml(&visible));
        p.push_str("\n\n");

        let inject = std::env::var("ROBONIX_SKILLS_INJECT_BODIES")
            .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
            .unwrap_or(false);
        if inject {
            p.push_str(&format_skill_playbooks(&visible));
            p.push('\n');
        } else {
            p.push_str(
                "Full playbooks are NOT inlined — use read_file on the `path` above before\n\
                 executing any skill-specific tool calls.\n\
                 (Set ROBONIX_SKILLS_INJECT_BODIES=1 to inline them.)\n",
            );
        }
    } else {
        p.push_str("## Skills index\n\nNo skills registered yet.\n");
    }

    // Hidden skills.
    let hidden: Vec<&AgentSkill> = skills
        .iter()
        .filter(|s| skill_disable_model_invocation(&s.metadata_json))
        .collect();
    if !hidden.is_empty() {
        p.push_str("\n## Skills hidden from model context\n\n");
        for s in hidden {
            p.push_str(&format!(
                "- **{}** (node={}): {} — `{}`\n",
                s.name, s.node_id, s.description, s.path
            ));
        }
    }

    p
}
