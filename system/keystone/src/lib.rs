// SPDX-License-Identifier: MulanPSL-2.0

//! Persistent identity and account management for Robonix.

pub mod pb;

use std::collections::BTreeSet;
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

use argon2::Argon2;
use argon2::password_hash::{PasswordHash, PasswordHasher, PasswordVerifier, SaltString};
use base64::Engine;
use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use rand::RngCore;
use rusqlite::{Connection, OptionalExtension, params};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use thiserror::Error;
use uuid::Uuid;

const DEFAULT_SESSION_TTL_MS: u64 = 24 * 60 * 60 * 1_000;
const MIN_PASSWORD_LENGTH: usize = 8;
const ROLE_USER: &str = "user";
const ROLE_ADMIN: &str = "admin";

/// Public account data. Password hashes and session identifiers never appear here.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct User {
    pub user_id: String,
    pub username: String,
    pub display_name: String,
    pub email: String,
    pub enabled: bool,
    pub roles: Vec<String>,
    pub voice_guard_enabled: bool,
    pub voiceprint_enrolled: bool,
    pub password_change_required: bool,
    pub created_at_ms: u64,
    pub updated_at_ms: u64,
}

impl User {
    pub fn is_admin(&self) -> bool {
        self.roles.iter().any(|role| role == ROLE_ADMIN)
    }
}

/// A newly issued, opaque server-side session.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthSession {
    pub token: String,
    pub expires_at_ms: u64,
    pub user: User,
}

/// A private, owner-readable voiceprint sample used for profile preview.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VoiceprintPreview {
    pub audio_data: Vec<u8>,
    pub sample_rate_hz: u32,
    pub updated_at_ms: u64,
}

/// Errors surfaced by Keystone's storage and authorization boundary.
#[derive(Debug, Error)]
pub enum KeystoneError {
    #[error("authentication failed")]
    AuthenticationFailed,
    #[error("administrator role is required")]
    AdminRequired,
    #[error("user is disabled")]
    UserDisabled,
    #[error("user '{0}' does not exist")]
    UserNotFound(String),
    #[error("username is already registered")]
    UsernameExists,
    #[error("public signup is disabled")]
    SignupDisabled,
    #[error("username must be 3-64 characters using letters, numbers, '.', '_' or '-'")]
    InvalidUsername,
    #[error("display name must not be empty")]
    InvalidDisplayName,
    #[error("email address is invalid")]
    InvalidEmail,
    #[error("password must contain at least {MIN_PASSWORD_LENGTH} characters")]
    WeakPassword,
    #[error("role '{0}' is not supported")]
    InvalidRole(String),
    #[error("at least one enabled administrator must remain")]
    LastAdmin,
    #[error("voiceprint subject must not be empty")]
    InvalidVoiceprintSubject,
    #[error("voiceprint is already bound to another user")]
    VoiceprintAlreadyBound,
    #[error("voiceprint does not match the logged-in user")]
    VoiceprintMismatch,
    #[error("voiceprint confidence is below the configured threshold")]
    VoiceprintConfidenceLow,
    #[error("password hashing failed: {0}")]
    PasswordHash(String),
    #[error("database lock is poisoned")]
    LockPoisoned,
    #[error(transparent)]
    Database(#[from] rusqlite::Error),
}

/// Thread-safe SQLite account store shared by the gRPC service.
#[derive(Clone)]
pub struct KeystoneStore {
    connection: Arc<Mutex<Connection>>,
    session_ttl_ms: u64,
}

impl KeystoneStore {
    /// Open or create a persistent Keystone database and apply the current schema.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, KeystoneError> {
        let connection = Connection::open(path)?;
        Self::from_connection(connection)
    }

    /// Create an isolated in-memory store for tests.
    pub fn open_in_memory() -> Result<Self, KeystoneError> {
        Self::from_connection(Connection::open_in_memory()?)
    }

    fn from_connection(connection: Connection) -> Result<Self, KeystoneError> {
        connection.execute_batch(
            r#"
            PRAGMA foreign_keys = ON;
            PRAGMA journal_mode = WAL;

            CREATE TABLE IF NOT EXISTS users (
                user_id TEXT PRIMARY KEY,
                username TEXT NOT NULL UNIQUE COLLATE NOCASE,
                display_name TEXT NOT NULL,
                email TEXT NOT NULL DEFAULT '',
                password_hash TEXT NOT NULL,
                enabled INTEGER NOT NULL DEFAULT 1,
                voice_guard_enabled INTEGER NOT NULL DEFAULT 0,
                password_change_required INTEGER NOT NULL DEFAULT 0,
                created_at_ms INTEGER NOT NULL,
                updated_at_ms INTEGER NOT NULL
            );

            CREATE TABLE IF NOT EXISTS user_roles (
                user_id TEXT NOT NULL REFERENCES users(user_id) ON DELETE CASCADE,
                role TEXT NOT NULL,
                PRIMARY KEY (user_id, role)
            );

            CREATE TABLE IF NOT EXISTS credentials (
                credential_id TEXT PRIMARY KEY,
                user_id TEXT NOT NULL REFERENCES users(user_id) ON DELETE CASCADE,
                kind TEXT NOT NULL,
                provider_id TEXT NOT NULL,
                external_subject_id TEXT NOT NULL,
                created_at_ms INTEGER NOT NULL,
                UNIQUE (kind, provider_id, external_subject_id),
                UNIQUE (user_id, kind, provider_id)
            );

            CREATE TABLE IF NOT EXISTS sessions (
                token_hash TEXT PRIMARY KEY,
                user_id TEXT NOT NULL REFERENCES users(user_id) ON DELETE CASCADE,
                created_at_ms INTEGER NOT NULL,
                expires_at_ms INTEGER NOT NULL,
                last_seen_at_ms INTEGER NOT NULL,
                revoked INTEGER NOT NULL DEFAULT 0
            );

            CREATE TABLE IF NOT EXISTS voiceprint_previews (
                user_id TEXT PRIMARY KEY REFERENCES users(user_id) ON DELETE CASCADE,
                audio_data BLOB NOT NULL,
                sample_rate_hz INTEGER NOT NULL,
                updated_at_ms INTEGER NOT NULL
            );

            CREATE TABLE IF NOT EXISTS settings (
                key TEXT PRIMARY KEY,
                value TEXT NOT NULL
            );

            INSERT OR IGNORE INTO settings(key, value)
            VALUES ('signup_enabled', 'true');
            "#,
        )?;
        Ok(Self {
            connection: Arc::new(Mutex::new(connection)),
            session_ttl_ms: DEFAULT_SESSION_TTL_MS,
        })
    }

    fn connection(&self) -> Result<std::sync::MutexGuard<'_, Connection>, KeystoneError> {
        self.connection
            .lock()
            .map_err(|_| KeystoneError::LockPoisoned)
    }

    /// Create the first administrator exactly once.
    pub fn bootstrap_admin(
        &self,
        username: &str,
        display_name: &str,
        email: &str,
        password: &str,
        password_change_required: bool,
    ) -> Result<Option<User>, KeystoneError> {
        validate_profile(username, display_name, email)?;
        validate_password(password)?;
        let password_hash = hash_password(password)?;
        let now = now_ms();
        let mut connection = self.connection()?;
        let transaction = connection.transaction()?;
        let count: u64 =
            transaction.query_row("SELECT COUNT(*) FROM users", [], |row| row.get(0))?;
        if count != 0 {
            return Ok(None);
        }
        let user_id = new_user_id();
        transaction.execute(
            "INSERT INTO users (
                user_id, username, display_name, email, password_hash, enabled,
                voice_guard_enabled, password_change_required, created_at_ms, updated_at_ms
             ) VALUES (?1, ?2, ?3, ?4, ?5, 1, 0, ?6, ?7, ?7)",
            params![
                user_id,
                normalize_username(username),
                display_name.trim(),
                email.trim(),
                password_hash,
                password_change_required,
                now
            ],
        )?;
        transaction.execute(
            "INSERT INTO user_roles(user_id, role) VALUES (?1, ?2), (?1, ?3)",
            params![user_id, ROLE_USER, ROLE_ADMIN],
        )?;
        transaction.commit()?;
        drop(connection);
        self.get_user(&user_id).map(Some)
    }

    /// Register a normal user when public signup is enabled.
    pub fn register(
        &self,
        username: &str,
        display_name: &str,
        email: &str,
        password: &str,
    ) -> Result<AuthSession, KeystoneError> {
        validate_profile(username, display_name, email)?;
        validate_password(password)?;
        if !self.signup_enabled()? {
            return Err(KeystoneError::SignupDisabled);
        }
        let password_hash = hash_password(password)?;
        let now = now_ms();
        let user_id = new_user_id();
        {
            let mut connection = self.connection()?;
            let transaction = connection.transaction()?;
            let result = transaction.execute(
                "INSERT INTO users (
                    user_id, username, display_name, email, password_hash,
                    enabled, voice_guard_enabled, password_change_required,
                    created_at_ms, updated_at_ms
                 ) VALUES (?1, ?2, ?3, ?4, ?5, 1, 0, 0, ?6, ?6)",
                params![
                    user_id,
                    normalize_username(username),
                    display_name.trim(),
                    email.trim(),
                    password_hash,
                    now
                ],
            );
            match result {
                Ok(_) => {}
                Err(error) if is_unique_constraint(&error) => {
                    return Err(KeystoneError::UsernameExists);
                }
                Err(error) => return Err(error.into()),
            }
            transaction.execute(
                "INSERT INTO user_roles(user_id, role) VALUES (?1, ?2)",
                params![user_id, ROLE_USER],
            )?;
            transaction.commit()?;
        }
        self.issue_session(&user_id)
    }

    /// Validate credentials and issue a new opaque session token.
    pub fn login(&self, username: &str, password: &str) -> Result<AuthSession, KeystoneError> {
        let normalized = normalize_username(username);
        let (user_id, password_hash, enabled): (String, String, bool) = self
            .connection()?
            .query_row(
                "SELECT user_id, password_hash, enabled FROM users WHERE username = ?1",
                params![normalized],
                |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?)),
            )
            .optional()?
            .ok_or(KeystoneError::AuthenticationFailed)?;
        if !verify_password(password, &password_hash)? {
            return Err(KeystoneError::AuthenticationFailed);
        }
        if !enabled {
            return Err(KeystoneError::UserDisabled);
        }
        self.issue_session(&user_id)
    }

    fn issue_session(&self, user_id: &str) -> Result<AuthSession, KeystoneError> {
        let mut token_bytes = [0_u8; 32];
        rand::rng().fill_bytes(&mut token_bytes);
        let token = URL_SAFE_NO_PAD.encode(token_bytes);
        let token_hash = hash_session_token(&token);
        let created_at_ms = now_ms();
        let expires_at_ms = created_at_ms.saturating_add(self.session_ttl_ms);
        self.connection()?.execute(
            "INSERT INTO sessions(
                token_hash, user_id, created_at_ms, expires_at_ms, last_seen_at_ms, revoked
             ) VALUES (?1, ?2, ?3, ?4, ?3, 0)",
            params![token_hash, user_id, created_at_ms, expires_at_ms],
        )?;
        Ok(AuthSession {
            token,
            expires_at_ms,
            user: self.get_user(user_id)?,
        })
    }

    /// Resolve a live session and update its last-seen timestamp.
    pub fn authenticate(&self, token: &str) -> Result<User, KeystoneError> {
        let connection = self.connection()?;
        authenticate_on(&connection, token)
    }

    /// Revoke the presented session. Logout is idempotent.
    pub fn logout(&self, token: &str) -> Result<(), KeystoneError> {
        self.connection()?.execute(
            "UPDATE sessions SET revoked = 1 WHERE token_hash = ?1",
            params![hash_session_token(token)],
        )?;
        Ok(())
    }

    pub fn get_user(&self, user_id: &str) -> Result<User, KeystoneError> {
        let connection = self.connection()?;
        user_by_id(&connection, user_id)?
            .ok_or_else(|| KeystoneError::UserNotFound(user_id.to_string()))
    }

    /// Update the logged-in user's editable profile fields.
    pub fn update_profile(
        &self,
        token: &str,
        display_name: &str,
        email: &str,
    ) -> Result<User, KeystoneError> {
        let user = self.authenticate(token)?;
        validate_display_name(display_name)?;
        validate_email(email)?;
        self.connection()?.execute(
            "UPDATE users
                SET display_name = ?2, email = ?3, updated_at_ms = ?4
              WHERE user_id = ?1",
            params![user.user_id, display_name.trim(), email.trim(), now_ms()],
        )?;
        self.get_user(&user.user_id)
    }

    /// Change the logged-in user's password and revoke every other session.
    pub fn change_password(
        &self,
        token: &str,
        current_password: &str,
        new_password: &str,
    ) -> Result<(), KeystoneError> {
        validate_password(new_password)?;
        let user = self.authenticate(token)?;
        let password_hash: String = self.connection()?.query_row(
            "SELECT password_hash FROM users WHERE user_id = ?1",
            params![user.user_id],
            |row| row.get(0),
        )?;
        if !verify_password(current_password, &password_hash)? {
            return Err(KeystoneError::AuthenticationFailed);
        }
        let new_hash = hash_password(new_password)?;
        let current_token_hash = hash_session_token(token);
        let mut connection = self.connection()?;
        let transaction = connection.transaction()?;
        transaction.execute(
            "UPDATE users
                SET password_hash = ?2, password_change_required = 0, updated_at_ms = ?3
              WHERE user_id = ?1",
            params![user.user_id, new_hash, now_ms()],
        )?;
        transaction.execute(
            "UPDATE sessions SET revoked = 1
              WHERE user_id = ?1 AND token_hash <> ?2",
            params![user.user_id, current_token_hash],
        )?;
        transaction.commit()?;
        Ok(())
    }

    pub fn list_users(&self, admin_token: &str) -> Result<Vec<User>, KeystoneError> {
        let connection = self.connection()?;
        require_admin_on(&connection, admin_token)?;
        let mut statement = connection.prepare("SELECT user_id FROM users ORDER BY username")?;
        let ids = statement
            .query_map([], |row| row.get::<_, String>(0))?
            .collect::<Result<Vec<_>, _>>()?;
        ids.into_iter()
            .map(|user_id| {
                user_by_id(&connection, &user_id)?
                    .ok_or_else(|| KeystoneError::UserNotFound(user_id))
            })
            .collect()
    }

    /// Apply administrator-owned account controls.
    pub fn admin_update_user(
        &self,
        admin_token: &str,
        target_user_id: &str,
        enabled: bool,
        roles: &[String],
        voice_guard_enabled: bool,
    ) -> Result<User, KeystoneError> {
        let normalized_roles = normalize_roles(roles)?;
        let mut connection = self.connection()?;
        let transaction = connection.transaction()?;
        require_admin_on(&transaction, admin_token)?;
        let target = user_by_id(&transaction, target_user_id)?
            .ok_or_else(|| KeystoneError::UserNotFound(target_user_id.to_string()))?;
        let removes_admin = target.is_admin() && !normalized_roles.contains(ROLE_ADMIN);
        if target.is_admin() && (!enabled || removes_admin) {
            ensure_another_enabled_admin_on(&transaction, target_user_id)?;
        }
        transaction.execute(
            "UPDATE users
                SET enabled = ?2, voice_guard_enabled = ?3, updated_at_ms = ?4
              WHERE user_id = ?1",
            params![target_user_id, enabled, voice_guard_enabled, now_ms()],
        )?;
        transaction.execute(
            "DELETE FROM user_roles WHERE user_id = ?1",
            params![target_user_id],
        )?;
        for role in normalized_roles {
            transaction.execute(
                "INSERT INTO user_roles(user_id, role) VALUES (?1, ?2)",
                params![target_user_id, role],
            )?;
        }
        if !enabled {
            transaction.execute(
                "UPDATE sessions SET revoked = 1 WHERE user_id = ?1",
                params![target_user_id],
            )?;
        }
        transaction.commit()?;
        drop(connection);
        self.get_user(target_user_id)
    }

    pub fn admin_delete_user(
        &self,
        admin_token: &str,
        target_user_id: &str,
    ) -> Result<(), KeystoneError> {
        let mut connection = self.connection()?;
        let transaction = connection.transaction()?;
        require_admin_on(&transaction, admin_token)?;
        let target = user_by_id(&transaction, target_user_id)?
            .ok_or_else(|| KeystoneError::UserNotFound(target_user_id.to_string()))?;
        if target.is_admin() {
            ensure_another_enabled_admin_on(&transaction, target_user_id)?;
        }
        let changed = transaction.execute(
            "DELETE FROM users WHERE user_id = ?1",
            params![target_user_id],
        )?;
        if changed == 0 {
            return Err(KeystoneError::UserNotFound(target_user_id.to_string()));
        }
        transaction.commit()?;
        Ok(())
    }

    /// Replace the logged-in user's voiceprint binding.
    pub fn bind_voiceprint(
        &self,
        token: &str,
        external_subject_id: &str,
    ) -> Result<User, KeystoneError> {
        let user = self.authenticate(token)?;
        self.bind_voiceprint_for_user(&user.user_id, external_subject_id)?;
        self.get_user(&user.user_id)
    }

    /// Replace a voiceprint binding and its private profile preview atomically.
    pub fn replace_voiceprint(
        &self,
        token: &str,
        external_subject_id: &str,
        audio_data: &[u8],
        sample_rate_hz: u32,
    ) -> Result<User, KeystoneError> {
        let user = self.authenticate(token)?;
        let subject = external_subject_id.trim();
        if subject.is_empty() {
            return Err(KeystoneError::InvalidVoiceprintSubject);
        }
        let now = now_ms();
        let mut connection = self.connection()?;
        let transaction = connection.transaction()?;
        transaction.execute(
            "DELETE FROM credentials
              WHERE user_id = ?1 AND kind = 'voiceprint' AND provider_id = 'voiceprint'",
            params![user.user_id],
        )?;
        let result = transaction.execute(
            "INSERT INTO credentials(
                credential_id, user_id, kind, provider_id, external_subject_id, created_at_ms
             ) VALUES (?1, ?2, 'voiceprint', 'voiceprint', ?3, ?4)",
            params![
                format!("credential_{}", Uuid::new_v4().simple()),
                user.user_id,
                subject,
                now
            ],
        );
        match result {
            Ok(_) => {}
            Err(error) if is_unique_constraint(&error) => {
                return Err(KeystoneError::VoiceprintAlreadyBound);
            }
            Err(error) => return Err(error.into()),
        }
        transaction.execute(
            "INSERT INTO voiceprint_previews(
                user_id, audio_data, sample_rate_hz, updated_at_ms
             ) VALUES (?1, ?2, ?3, ?4)
             ON CONFLICT(user_id) DO UPDATE SET
                audio_data = excluded.audio_data,
                sample_rate_hz = excluded.sample_rate_hz,
                updated_at_ms = excluded.updated_at_ms",
            params![user.user_id, audio_data, sample_rate_hz, now],
        )?;
        transaction.commit()?;
        drop(connection);
        self.get_user(&user.user_id)
    }

    /// Read only the logged-in account's saved voiceprint preview.
    pub fn voiceprint_preview(
        &self,
        token: &str,
    ) -> Result<Option<VoiceprintPreview>, KeystoneError> {
        let user = self.authenticate(token)?;
        self.connection()?
            .query_row(
                "SELECT audio_data, sample_rate_hz, updated_at_ms
                   FROM voiceprint_previews
                  WHERE user_id = ?1",
                params![user.user_id],
                |row| {
                    Ok(VoiceprintPreview {
                        audio_data: row.get(0)?,
                        sample_rate_hz: row.get(1)?,
                        updated_at_ms: row.get(2)?,
                    })
                },
            )
            .optional()
            .map_err(Into::into)
    }

    fn bind_voiceprint_for_user(
        &self,
        user_id: &str,
        external_subject_id: &str,
    ) -> Result<(), KeystoneError> {
        let subject = external_subject_id.trim();
        if subject.is_empty() {
            return Err(KeystoneError::InvalidVoiceprintSubject);
        }
        let now = now_ms();
        let mut connection = self.connection()?;
        let transaction = connection.transaction()?;
        transaction.execute(
            "DELETE FROM credentials
              WHERE user_id = ?1 AND kind = 'voiceprint' AND provider_id = 'voiceprint'",
            params![user_id],
        )?;
        let result = transaction.execute(
            "INSERT INTO credentials(
                credential_id, user_id, kind, provider_id, external_subject_id, created_at_ms
             ) VALUES (?1, ?2, 'voiceprint', 'voiceprint', ?3, ?4)",
            params![
                format!("credential_{}", Uuid::new_v4().simple()),
                user_id,
                subject,
                now
            ],
        );
        match result {
            Ok(_) => transaction.commit()?,
            Err(error) if is_unique_constraint(&error) => {
                return Err(KeystoneError::VoiceprintAlreadyBound);
            }
            Err(error) => return Err(error.into()),
        }
        Ok(())
    }

    pub fn unbind_voiceprint(&self, token: &str) -> Result<User, KeystoneError> {
        let user = self.authenticate(token)?;
        self.delete_voiceprint_for_user(&user.user_id)?;
        self.get_user(&user.user_id)
    }

    pub fn admin_reset_voiceprint(
        &self,
        admin_token: &str,
        target_user_id: &str,
    ) -> Result<User, KeystoneError> {
        let mut connection = self.connection()?;
        let transaction = connection.transaction()?;
        require_admin_on(&transaction, admin_token)?;
        user_by_id(&transaction, target_user_id)?
            .ok_or_else(|| KeystoneError::UserNotFound(target_user_id.to_string()))?;
        transaction.execute(
            "DELETE FROM credentials
              WHERE user_id = ?1 AND kind = 'voiceprint' AND provider_id = 'voiceprint'",
            params![target_user_id],
        )?;
        transaction.execute(
            "DELETE FROM voiceprint_previews WHERE user_id = ?1",
            params![target_user_id],
        )?;
        transaction.commit()?;
        drop(connection);
        self.get_user(target_user_id)
    }

    fn delete_voiceprint_for_user(&self, user_id: &str) -> Result<(), KeystoneError> {
        let mut connection = self.connection()?;
        let transaction = connection.transaction()?;
        transaction.execute(
            "DELETE FROM credentials
              WHERE user_id = ?1 AND kind = 'voiceprint' AND provider_id = 'voiceprint'",
            params![user_id],
        )?;
        transaction.execute(
            "DELETE FROM voiceprint_previews WHERE user_id = ?1",
            params![user_id],
        )?;
        transaction.commit()?;
        Ok(())
    }

    /// Enforce the per-user voice guard for one voice turn.
    pub fn verify_voice(
        &self,
        token: &str,
        external_subject_id: &str,
        confidence: f32,
        minimum_confidence: f32,
    ) -> Result<(User, bool), KeystoneError> {
        let user = self.authenticate(token)?;
        if !user.voice_guard_enabled {
            return Ok((user, false));
        }
        if confidence < minimum_confidence {
            return Err(KeystoneError::VoiceprintConfidenceLow);
        }
        let owner: Option<String> = self
            .connection()?
            .query_row(
                "SELECT user_id FROM credentials
                  WHERE kind = 'voiceprint'
                    AND provider_id = 'voiceprint'
                    AND external_subject_id = ?1",
                params![external_subject_id],
                |row| row.get(0),
            )
            .optional()?;
        if owner.as_deref() != Some(user.user_id.as_str()) {
            return Err(KeystoneError::VoiceprintMismatch);
        }
        Ok((user, true))
    }

    pub fn signup_enabled(&self) -> Result<bool, KeystoneError> {
        let value: String = self.connection()?.query_row(
            "SELECT value FROM settings WHERE key = 'signup_enabled'",
            [],
            |row| row.get(0),
        )?;
        Ok(value == "true")
    }

    pub fn set_signup_enabled(
        &self,
        admin_token: &str,
        enabled: bool,
    ) -> Result<bool, KeystoneError> {
        let mut connection = self.connection()?;
        let transaction = connection.transaction()?;
        require_admin_on(&transaction, admin_token)?;
        transaction.execute(
            "INSERT INTO settings(key, value) VALUES ('signup_enabled', ?1)
             ON CONFLICT(key) DO UPDATE SET value = excluded.value",
            params![if enabled { "true" } else { "false" }],
        )?;
        transaction.commit()?;
        Ok(enabled)
    }
}

fn authenticate_on(connection: &Connection, token: &str) -> Result<User, KeystoneError> {
    if token.is_empty() {
        return Err(KeystoneError::AuthenticationFailed);
    }
    let token_hash = hash_session_token(token);
    let now = now_ms();
    let user_id: String = connection
        .query_row(
            "SELECT s.user_id
               FROM sessions s
               JOIN users u ON u.user_id = s.user_id
              WHERE s.token_hash = ?1
                AND s.revoked = 0
                AND s.expires_at_ms > ?2
                AND u.enabled = 1",
            params![token_hash, now],
            |row| row.get(0),
        )
        .optional()?
        .ok_or(KeystoneError::AuthenticationFailed)?;
    connection.execute(
        "UPDATE sessions SET last_seen_at_ms = ?2 WHERE token_hash = ?1",
        params![token_hash, now],
    )?;
    user_by_id(connection, &user_id)?
        .ok_or_else(|| KeystoneError::UserNotFound(user_id.to_string()))
}

fn require_admin_on(connection: &Connection, token: &str) -> Result<User, KeystoneError> {
    let user = authenticate_on(connection, token)?;
    if !user.is_admin() {
        return Err(KeystoneError::AdminRequired);
    }
    Ok(user)
}

fn ensure_another_enabled_admin_on(
    connection: &Connection,
    excluded_user_id: &str,
) -> Result<(), KeystoneError> {
    let count: u64 = connection.query_row(
        "SELECT COUNT(DISTINCT u.user_id)
           FROM users u
           JOIN user_roles r ON r.user_id = u.user_id
          WHERE u.enabled = 1 AND r.role = ?1 AND u.user_id <> ?2",
        params![ROLE_ADMIN, excluded_user_id],
        |row| row.get(0),
    )?;
    if count == 0 {
        return Err(KeystoneError::LastAdmin);
    }
    Ok(())
}

fn user_by_id(connection: &Connection, user_id: &str) -> Result<Option<User>, KeystoneError> {
    type UserRow = (String, String, String, String, bool, bool, bool, u64, u64);
    let row: Option<UserRow> = connection
        .query_row(
            "SELECT user_id, username, display_name, email, enabled,
                    voice_guard_enabled, password_change_required,
                    created_at_ms, updated_at_ms
               FROM users WHERE user_id = ?1",
            params![user_id],
            |row| {
                Ok((
                    row.get(0)?,
                    row.get(1)?,
                    row.get(2)?,
                    row.get(3)?,
                    row.get(4)?,
                    row.get(5)?,
                    row.get(6)?,
                    row.get(7)?,
                    row.get(8)?,
                ))
            },
        )
        .optional()?;
    let Some((
        user_id,
        username,
        display_name,
        email,
        enabled,
        voice_guard_enabled,
        password_change_required,
        created_at_ms,
        updated_at_ms,
    )) = row
    else {
        return Ok(None);
    };
    let mut statement =
        connection.prepare("SELECT role FROM user_roles WHERE user_id = ?1 ORDER BY role")?;
    let roles = statement
        .query_map(params![user_id], |row| row.get::<_, String>(0))?
        .collect::<Result<Vec<_>, _>>()?;
    let voiceprint_enrolled: bool = connection.query_row(
        "SELECT EXISTS(
            SELECT 1 FROM credentials
             WHERE user_id = ?1 AND kind = 'voiceprint' AND provider_id = 'voiceprint'
         )",
        params![user_id],
        |row| row.get(0),
    )?;
    Ok(Some(User {
        user_id,
        username,
        display_name,
        email,
        enabled,
        roles,
        voice_guard_enabled,
        voiceprint_enrolled,
        password_change_required,
        created_at_ms,
        updated_at_ms,
    }))
}

fn normalize_username(username: &str) -> String {
    username.trim().to_ascii_lowercase()
}

fn validate_profile(username: &str, display_name: &str, email: &str) -> Result<(), KeystoneError> {
    let username = username.trim();
    if !(3..=64).contains(&username.len())
        || !username
            .chars()
            .all(|ch| ch.is_ascii_alphanumeric() || matches!(ch, '.' | '_' | '-'))
    {
        return Err(KeystoneError::InvalidUsername);
    }
    validate_display_name(display_name)?;
    validate_email(email)
}

fn validate_display_name(display_name: &str) -> Result<(), KeystoneError> {
    if display_name.trim().is_empty() {
        return Err(KeystoneError::InvalidDisplayName);
    }
    Ok(())
}

fn validate_email(email: &str) -> Result<(), KeystoneError> {
    let email = email.trim();
    if !email.is_empty()
        && (email.contains(char::is_whitespace)
            || !email.contains('@')
            || email.starts_with('@')
            || email.ends_with('@'))
    {
        return Err(KeystoneError::InvalidEmail);
    }
    Ok(())
}

fn validate_password(password: &str) -> Result<(), KeystoneError> {
    if password.chars().count() < MIN_PASSWORD_LENGTH {
        return Err(KeystoneError::WeakPassword);
    }
    Ok(())
}

fn normalize_roles(roles: &[String]) -> Result<BTreeSet<String>, KeystoneError> {
    let mut normalized = BTreeSet::new();
    for role in roles {
        let role = role.trim().to_ascii_lowercase();
        if !matches!(role.as_str(), ROLE_USER | ROLE_ADMIN) {
            return Err(KeystoneError::InvalidRole(role));
        }
        normalized.insert(role);
    }
    normalized.insert(ROLE_USER.to_string());
    Ok(normalized)
}

fn hash_password(password: &str) -> Result<String, KeystoneError> {
    let mut salt_bytes = [0_u8; 16];
    rand::rng().fill_bytes(&mut salt_bytes);
    let salt = SaltString::encode_b64(&salt_bytes)
        .map_err(|error| KeystoneError::PasswordHash(error.to_string()))?;
    Argon2::default()
        .hash_password(password.as_bytes(), &salt)
        .map(|hash| hash.to_string())
        .map_err(|error| KeystoneError::PasswordHash(error.to_string()))
}

fn verify_password(password: &str, encoded: &str) -> Result<bool, KeystoneError> {
    let parsed = PasswordHash::new(encoded)
        .map_err(|error| KeystoneError::PasswordHash(error.to_string()))?;
    Ok(Argon2::default()
        .verify_password(password.as_bytes(), &parsed)
        .is_ok())
}

fn hash_session_token(token: &str) -> String {
    let digest = Sha256::digest(token.as_bytes());
    URL_SAFE_NO_PAD.encode(digest)
}

fn new_user_id() -> String {
    format!("user_{}", Uuid::new_v4().simple())
}

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
        .try_into()
        .unwrap_or(u64::MAX)
}

fn is_unique_constraint(error: &rusqlite::Error) -> bool {
    matches!(
        error,
        rusqlite::Error::SqliteFailure(code, _)
            if code.code == rusqlite::ErrorCode::ConstraintViolation
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    const ADMIN_PASSWORD: &str = "admin-password";
    const USER_PASSWORD: &str = "user-password";

    fn store_with_admin() -> (KeystoneStore, AuthSession) {
        let store = KeystoneStore::open_in_memory().expect("store");
        store
            .bootstrap_admin(
                "admin",
                "Administrator",
                "admin@example.com",
                ADMIN_PASSWORD,
                true,
            )
            .expect("bootstrap")
            .expect("created");
        let admin = store.login("admin", ADMIN_PASSWORD).expect("admin login");
        (store, admin)
    }

    #[test]
    fn register_login_profile_and_restart_persist() {
        let directory = tempfile::tempdir().expect("tempdir");
        let database = directory.path().join("keystone.db");
        let user_id;
        {
            let store = KeystoneStore::open(&database).expect("open");
            store
                .bootstrap_admin("admin", "Admin", "", ADMIN_PASSWORD, true)
                .expect("bootstrap");
            let session = store
                .register("alice", "Alice", "alice@example.com", USER_PASSWORD)
                .expect("register");
            user_id = session.user.user_id.clone();
            let updated = store
                .update_profile(&session.token, "Alice Smith", "new@example.com")
                .expect("update");
            assert_eq!(updated.display_name, "Alice Smith");
            assert_eq!(updated.email, "new@example.com");
        }
        let reopened = KeystoneStore::open(&database).expect("reopen");
        let user = reopened.get_user(&user_id).expect("persisted user");
        assert_eq!(user.display_name, "Alice Smith");
        assert!(reopened.login("alice", USER_PASSWORD).is_ok());
    }

    #[test]
    fn normal_user_cannot_administer_accounts() {
        let (store, _) = store_with_admin();
        let user = store
            .register("alice", "Alice", "", USER_PASSWORD)
            .expect("register");
        let result = store.list_users(&user.token);
        assert!(matches!(result, Err(KeystoneError::AdminRequired)));
    }

    #[test]
    fn admin_can_promote_user_but_cannot_remove_last_admin() {
        let (store, admin) = store_with_admin();
        let user = store
            .register("alice", "Alice", "", USER_PASSWORD)
            .expect("register");
        let promoted = store
            .admin_update_user(
                &admin.token,
                &user.user.user_id,
                true,
                &["admin".to_string()],
                true,
            )
            .expect("promote");
        assert!(promoted.is_admin());
        assert!(promoted.voice_guard_enabled);

        let demoted_admin = store
            .admin_update_user(
                &admin.token,
                &admin.user.user_id,
                true,
                &["user".to_string()],
                false,
            )
            .expect("another admin remains");
        assert!(!demoted_admin.is_admin());

        let promoted_login = store.login("alice", USER_PASSWORD).expect("login");
        let error = store
            .admin_update_user(
                &promoted_login.token,
                &promoted.user_id,
                false,
                &["user".to_string()],
                false,
            )
            .expect_err("last admin protected");
        assert!(matches!(error, KeystoneError::LastAdmin));
    }

    #[test]
    fn concurrent_admin_demotion_keeps_one_enabled_administrator() {
        use std::sync::{Arc, Barrier};

        let (store, admin) = store_with_admin();
        let user = store
            .register("alice", "Alice", "", USER_PASSWORD)
            .expect("register");
        let promoted = store
            .admin_update_user(
                &admin.token,
                &user.user.user_id,
                true,
                &["admin".to_string()],
                false,
            )
            .expect("promote");
        let promoted_session = store.login("alice", USER_PASSWORD).expect("promoted login");
        let barrier = Arc::new(Barrier::new(3));

        let first_store = store.clone();
        let first_barrier = barrier.clone();
        let first_token = admin.token.clone();
        let first_target = admin.user.user_id.clone();
        let first = std::thread::spawn(move || {
            first_barrier.wait();
            first_store.admin_update_user(
                &first_token,
                &first_target,
                true,
                &["user".to_string()],
                false,
            )
        });

        let second_store = store.clone();
        let second_barrier = barrier.clone();
        let second_token = promoted_session.token;
        let second_target = promoted.user_id;
        let second = std::thread::spawn(move || {
            second_barrier.wait();
            second_store.admin_update_user(
                &second_token,
                &second_target,
                true,
                &["user".to_string()],
                false,
            )
        });

        barrier.wait();
        let results = [first.join().expect("first"), second.join().expect("second")];
        assert_eq!(results.iter().filter(|result| result.is_ok()).count(), 1);
        let remaining_admins = [admin.user.user_id, user.user.user_id]
            .into_iter()
            .filter(|user_id| store.get_user(user_id).expect("user").is_admin())
            .count();
        assert_eq!(remaining_admins, 1);
    }

    #[test]
    fn voice_guard_matches_only_the_logged_in_user() {
        let (store, admin) = store_with_admin();
        let alice = store
            .register("alice", "Alice", "", USER_PASSWORD)
            .expect("alice");
        let bob = store
            .register("bob", "Bob", "", USER_PASSWORD)
            .expect("bob");
        store
            .bind_voiceprint(&alice.token, "voice-alice")
            .expect("bind alice");
        store
            .bind_voiceprint(&bob.token, "voice-bob")
            .expect("bind bob");
        store
            .admin_update_user(
                &admin.token,
                &alice.user.user_id,
                true,
                &["user".to_string()],
                true,
            )
            .expect("enable guard");

        let (_, verified) = store
            .verify_voice(&alice.token, "voice-alice", 0.91, 0.75)
            .expect("matching voice");
        assert!(verified);
        assert!(matches!(
            store.verify_voice(&alice.token, "voice-bob", 0.91, 0.75),
            Err(KeystoneError::VoiceprintMismatch)
        ));
        assert!(matches!(
            store.verify_voice(&alice.token, "voice-alice", 0.5, 0.75),
            Err(KeystoneError::VoiceprintConfidenceLow)
        ));
    }

    #[test]
    fn voiceprint_preview_is_private_and_removed_with_binding() {
        let (store, _) = store_with_admin();
        let alice = store
            .register("alice", "Alice", "", USER_PASSWORD)
            .expect("alice");
        let bob = store
            .register("bob", "Bob", "", USER_PASSWORD)
            .expect("bob");
        let audio = vec![0_u8, 1, 2, 3, 4, 5];

        store
            .replace_voiceprint(&alice.token, "voice-alice", &audio, 16_000)
            .expect("replace");
        assert_eq!(
            store
                .voiceprint_preview(&alice.token)
                .expect("alice preview")
                .expect("preview")
                .audio_data,
            audio
        );
        assert_eq!(
            store.voiceprint_preview(&bob.token).expect("bob preview"),
            None
        );

        store.unbind_voiceprint(&alice.token).expect("unbind");
        assert_eq!(
            store
                .voiceprint_preview(&alice.token)
                .expect("preview removed"),
            None
        );
    }

    #[test]
    fn password_change_revokes_other_sessions() {
        let (store, _) = store_with_admin();
        let first = store
            .register("alice", "Alice", "", USER_PASSWORD)
            .expect("register");
        let second = store.login("alice", USER_PASSWORD).expect("login again");
        store
            .change_password(&first.token, USER_PASSWORD, "new-password")
            .expect("change");
        assert!(store.authenticate(&first.token).is_ok());
        assert!(matches!(
            store.authenticate(&second.token),
            Err(KeystoneError::AuthenticationFailed)
        ));
        assert!(store.login("alice", "new-password").is_ok());
    }
}
