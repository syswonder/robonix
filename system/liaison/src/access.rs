// SPDX-License-Identifier: MulanPSL-2.0
//! Liaison-local access gate for text and voice entry.

use std::collections::BTreeSet;

use crate::pb::voiceprint;

pub const DEFAULT_VOICE_THRESHOLD: f32 = 0.25;

#[derive(Clone, Debug, PartialEq)]
pub enum AccessDecision {
    Allow {
        user_id: String,
        method: AccessMethod,
        confidence: f32,
        reason: String,
    },
    Deny {
        user_id: String,
        confidence: f32,
        reason: String,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AccessMethod {
    UserList,
    Voiceprint,
    Disabled,
}

impl AccessMethod {
    /// Return the stable string written into `Task.context_json.access.method`
    /// so downstream logs can distinguish allow-list and voiceprint grants.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::UserList => "user_list",
            Self::Voiceprint => "voiceprint",
            Self::Disabled => "disabled",
        }
    }
}

#[derive(Clone, Debug)]
pub struct AccessControlConfig {
    pub enabled: bool,
    pub allowed_users: BTreeSet<String>,
    pub voice_threshold: f32,
}

impl AccessControlConfig {
    /// Build access settings from environment variables. The accepted user
    /// list is a comma/semicolon/whitespace separated set of canonical ids:
    /// `local:<os_user>` for text/client hints, `voice:<enrolled_id>` for
    /// speakers returned by voiceprint.
    pub fn from_env() -> Self {
        let enabled = env_bool("ROBONIX_LIAISON_ACCESS_ENABLED", false);
        let allowed_users =
            parse_user_set(&std::env::var("ROBONIX_LIAISON_ALLOWED_USERS").unwrap_or_default());
        let voice_threshold = std::env::var("ROBONIX_LIAISON_VOICE_THRESHOLD")
            .ok()
            .and_then(|s| s.parse::<f32>().ok())
            .unwrap_or(DEFAULT_VOICE_THRESHOLD);
        Self {
            enabled,
            allowed_users,
            voice_threshold,
        }
    }

    /// Decide whether a text/API task may enter Liaison. The caller should
    /// pass the already-normalized `context_json.user_id`; empty ids are denied
    /// when the gate is enabled because there is no identity to authorize.
    pub fn authorize_user(&self, user_id: &str) -> AccessDecision {
        let user_id = normalize_user_id(user_id, "local");
        if !self.enabled {
            return AccessDecision::Allow {
                user_id,
                method: AccessMethod::Disabled,
                confidence: 1.0,
                reason: "access gate disabled".to_string(),
            };
        }
        if self.allowed_users.contains(&user_id) {
            AccessDecision::Allow {
                user_id,
                method: AccessMethod::UserList,
                confidence: 1.0,
                reason: "user is in allowed user list".to_string(),
            }
        } else {
            AccessDecision::Deny {
                user_id,
                confidence: 0.0,
                reason: "user is not in allowed user list".to_string(),
            }
        }
    }

    /// Decide whether a voice turn may continue past capture and voiceprint.
    /// A caller hint in the allowed list is enough; otherwise the voiceprint
    /// response must identify an enrolled speaker above the configured
    /// threshold and that `voice:<id>` must also be allowed.
    pub fn authorize_voice(
        &self,
        client_user_id: &str,
        voiceprint: Option<&voiceprint::IdentifyResponse>,
    ) -> AccessDecision {
        if !self.enabled {
            return AccessDecision::Allow {
                user_id: voiceprint_user_id(voiceprint)
                    .unwrap_or_else(|| fallback_voice_user(client_user_id)),
                method: AccessMethod::Disabled,
                confidence: voiceprint.map(|r| r.confidence).unwrap_or(0.0),
                reason: "access gate disabled".to_string(),
            };
        }

        let hinted_user = normalize_user_id(client_user_id, "local");
        if !hinted_user.is_empty() && self.allowed_users.contains(&hinted_user) {
            return AccessDecision::Allow {
                user_id: hinted_user,
                method: AccessMethod::UserList,
                confidence: 1.0,
                reason: "client user is in allowed user list".to_string(),
            };
        }

        if let Some(resp) = voiceprint
            && let Some(voice_user) = voiceprint_user_id(Some(resp))
        {
            if resp.is_known
                && resp.confidence >= self.voice_threshold
                && self.allowed_users.contains(&voice_user)
            {
                return AccessDecision::Allow {
                    user_id: voice_user,
                    method: AccessMethod::Voiceprint,
                    confidence: resp.confidence,
                    reason: "voiceprint matched allowed enrolled speaker".to_string(),
                };
            }
            return AccessDecision::Deny {
                user_id: voice_user,
                confidence: resp.confidence,
                reason: format!(
                    "voiceprint not allowed or below threshold {:.2}",
                    self.voice_threshold
                ),
            };
        }

        AccessDecision::Deny {
            user_id: hinted_user,
            confidence: 0.0,
            reason: "no allowed user hint and no matching enrolled voiceprint".to_string(),
        }
    }
}

/// Normalize bare user ids into Liaison's canonical prefixed form. Existing
/// ids that already contain a prefix separator are preserved verbatim.
pub fn normalize_user_id(raw: &str, default_prefix: &str) -> String {
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return String::new();
    }
    if trimmed.contains(':') {
        trimmed.to_string()
    } else {
        format!("{default_prefix}:{trimmed}")
    }
}

/// Return the identity used while access control is disabled or while a
/// voiceprint provider is unavailable. This keeps old deployments working
/// without granting access when the gate is explicitly enabled.
fn fallback_voice_user(client_user_id: &str) -> String {
    let hinted = normalize_user_id(client_user_id, "local");
    if hinted.is_empty() {
        "voice:unknown".to_string()
    } else {
        hinted
    }
}

/// Convert a voiceprint response into Liaison's canonical `voice:<id>` form.
/// Empty or absent voiceprint ids return `None` so callers can distinguish
/// "unknown speaker" from a real enrolled id.
fn voiceprint_user_id(resp: Option<&voiceprint::IdentifyResponse>) -> Option<String> {
    let resp = resp?;
    if resp.user_id.trim().is_empty() {
        None
    } else {
        Some(normalize_user_id(&resp.user_id, "voice"))
    }
}

/// Parse a deploy/env user allowlist into canonical ids. Bare ids are treated
/// as local text users; voice ids must be written as `voice:<id>` to avoid
/// accidentally admitting a speaker when the operator meant an OS user.
fn parse_user_set(raw: &str) -> BTreeSet<String> {
    raw.split(|c: char| c == ',' || c == ';' || c.is_whitespace())
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(|s| {
            if s.contains(':') {
                s.to_string()
            } else {
                normalize_user_id(s, "local")
            }
        })
        .collect()
}

/// Read common boolean environment spellings while preserving a caller-supplied
/// default for unset values.
fn env_bool(key: &str, default: bool) -> bool {
    std::env::var(key)
        .ok()
        .map(|v| matches!(v.as_str(), "1" | "true" | "yes" | "on"))
        .unwrap_or(default)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn enabled_with(users: &[&str]) -> AccessControlConfig {
        AccessControlConfig {
            enabled: true,
            allowed_users: users.iter().map(|u| (*u).to_string()).collect(),
            voice_threshold: 0.7,
        }
    }

    #[test]
    fn text_user_must_be_allowed_when_enabled() {
        let cfg = enabled_with(&["local:liukaile"]);
        assert!(matches!(
            cfg.authorize_user("liukaile"),
            AccessDecision::Allow {
                method: AccessMethod::UserList,
                ..
            }
        ));
        assert!(matches!(
            cfg.authorize_user("local:bob"),
            AccessDecision::Deny { .. }
        ));
    }

    #[test]
    fn voice_allows_allowed_hint_before_voiceprint() {
        let cfg = enabled_with(&["local:liukaile"]);
        assert!(matches!(
            cfg.authorize_voice("liukaile", None),
            AccessDecision::Allow { user_id, method: AccessMethod::UserList, .. }
            if user_id == "local:liukaile"
        ));
    }

    #[test]
    fn voice_requires_known_allowed_speaker_above_threshold() {
        let cfg = enabled_with(&["voice:alice"]);
        let resp = voiceprint::IdentifyResponse {
            user_id: "alice".to_string(),
            user_name: "Alice".to_string(),
            confidence: 0.8,
            is_known: true,
            error: String::new(),
        };
        assert!(matches!(
            cfg.authorize_voice("", Some(&resp)),
            AccessDecision::Allow { user_id, method: AccessMethod::Voiceprint, .. }
            if user_id == "voice:alice"
        ));
    }

    #[test]
    fn voice_rejects_unknown_or_low_confidence() {
        let cfg = enabled_with(&["voice:alice"]);
        let resp = voiceprint::IdentifyResponse {
            user_id: "alice".to_string(),
            user_name: "Alice".to_string(),
            confidence: 0.4,
            is_known: true,
            error: String::new(),
        };
        assert!(matches!(
            cfg.authorize_voice("", Some(&resp)),
            AccessDecision::Deny { .. }
        ));
    }
}
