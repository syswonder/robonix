// SPDX-License-Identifier: MulanPSL-2.0

use std::fs::OpenOptions;
use std::io::{self, IsTerminal, Write};
use std::path::Path;

use anyhow::{Context, Result};
use crossterm::event::{self, Event, KeyCode, KeyModifiers};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use serde::{Deserialize, Serialize};
use tonic::{Code, Request};

use crate::pb::contracts::robonix_system_keystone_change_password_client::RobonixSystemKeystoneChangePasswordClient;
use crate::pb::contracts::robonix_system_keystone_get_profile_client::RobonixSystemKeystoneGetProfileClient;
use crate::pb::contracts::robonix_system_keystone_login_client::RobonixSystemKeystoneLoginClient;
use crate::pb::contracts::robonix_system_keystone_logout_client::RobonixSystemKeystoneLogoutClient;
use crate::pb::keystone::{
    ChangePasswordRequest, GetProfileRequest, LoginRequest, LogoutRequest, User,
};
use robonix_cli::Config;

const LOGIN_CONTRACT: &str = "robonix/system/keystone/login";
const LOGOUT_CONTRACT: &str = "robonix/system/keystone/logout";
const GET_PROFILE_CONTRACT: &str = "robonix/system/keystone/get_profile";
const CHANGE_PASSWORD_CONTRACT: &str = "robonix/system/keystone/change_password";
const CONSUMER_ID: &str = "rbnx-cli/auth";

pub struct Session {
    pub user_id: String,
    pub username: String,
    pub session_token: String,
}

#[derive(Deserialize, Serialize)]
struct StoredSession {
    atlas_endpoint: String,
    session_token: String,
    expires_at_ms: u64,
}

enum SessionResolution {
    Ready(Session),
    LoginRequired(String),
}

pub async fn resolve(server: &str, allow_login: bool) -> Result<Session> {
    match resolve_existing(server).await? {
        SessionResolution::Ready(session) => Ok(session),
        SessionResolution::LoginRequired(reason) => {
            if allow_login && environment_session_token().is_none() {
                eprintln!("{reason}");
                eprintln!("Log in to continue.");
                login(server, None).await?;
                match resolve_existing(server).await? {
                    SessionResolution::Ready(session) => Ok(session),
                    SessionResolution::LoginRequired(reason) => anyhow::bail!("{reason}"),
                }
            } else {
                anyhow::bail!("{reason}")
            }
        }
    }
}

async fn resolve_existing(server: &str) -> Result<SessionResolution> {
    let endpoint = super::canonical_endpoint(server);
    let mut atlas = AtlasClient::connect(&endpoint)
        .await
        .with_context(|| format!("connect to atlas at '{server}'"))?;
    if !has_keystone(&mut atlas).await? {
        let username = local_username();
        return Ok(SessionResolution::Ready(Session {
            user_id: format!("local:{username}"),
            username,
            session_token: String::new(),
        }));
    }

    let environment_token = environment_session_token();
    let stored = if environment_token.is_none() {
        let stored = load_session()?.filter(|session| session.atlas_endpoint == endpoint);
        if stored.as_ref().is_some_and(session_is_expired) {
            remove_session()?;
            None
        } else {
            stored
        }
    } else {
        None
    };
    let token = match environment_token
        .clone()
        .or_else(|| stored.as_ref().map(|session| session.session_token.clone()))
    {
        Some(token) => token,
        None => {
            return Ok(SessionResolution::LoginRequired(format!(
                "Keystone is enabled, but no saved session exists for this Atlas endpoint. \
                 Run `rbnx login --server {server}` or set ROBONIX_SESSION_TOKEN."
            )));
        }
    };

    let user = match get_profile(&mut atlas, &token).await {
        Ok(user) => user,
        Err(error) if is_password_change_required(&error) => {
            let reason = if environment_token.is_some() {
                "ROBONIX_SESSION_TOKEN requires a password change. Run `rbnx login` with that \
                 variable unset, then replace the token."
            } else {
                "Keystone requires a password change. Run `rbnx login` to change it."
            };
            return Ok(SessionResolution::LoginRequired(reason.to_string()));
        }
        Err(error) if is_authentication_failure(&error) => {
            let reason = if environment_token.is_some() {
                "ROBONIX_SESSION_TOKEN is not valid. Replace it or run `rbnx login` with that \
                 variable unset."
            } else {
                remove_session()?;
                "Keystone login session is not valid and the saved session was removed. \
                 Run `rbnx login` to sign in again."
            };
            return Ok(SessionResolution::LoginRequired(reason.to_string()));
        }
        Err(error) => {
            return Err(error.context("validate Keystone login session"));
        }
    };
    if user.password_change_required {
        let reason = if environment_token.is_some() {
            "ROBONIX_SESSION_TOKEN requires a password change. Run `rbnx login` with that \
             variable unset, then replace the token."
        } else {
            "Keystone requires a password change. Run `rbnx login` to change it."
        };
        return Ok(SessionResolution::LoginRequired(reason.to_string()));
    }
    Ok(SessionResolution::Ready(Session {
        user_id: user.user_id,
        username: user.username,
        session_token: token,
    }))
}

fn environment_session_token() -> Option<String> {
    std::env::var("ROBONIX_SESSION_TOKEN")
        .ok()
        .filter(|token| !token.is_empty())
}

fn logout_tokens(environment_token: Option<String>, stored: Option<&StoredSession>) -> Vec<String> {
    let mut tokens = Vec::new();
    if let Some(token) = environment_token {
        tokens.push(token);
    }
    if let Some(session) = stored
        && !tokens.iter().any(|token| token == &session.session_token)
    {
        tokens.push(session.session_token.clone());
    }
    tokens
}

fn is_authentication_failure(error: &anyhow::Error) -> bool {
    error.chain().any(|cause| {
        cause
            .downcast_ref::<tonic::Status>()
            .is_some_and(|status| status.code() == Code::Unauthenticated)
    })
}

fn is_password_change_required(error: &anyhow::Error) -> bool {
    error.chain().any(|cause| {
        cause
            .downcast_ref::<tonic::Status>()
            .is_some_and(|status| status.code() == Code::FailedPrecondition)
    })
}

pub async fn login(server: &str, username: Option<String>) -> Result<()> {
    let endpoint = super::canonical_endpoint(server);
    let username = match username {
        Some(username) if !username.trim().is_empty() => username,
        _ => prompt_line("Username: ")?,
    };
    let password = prompt_password("Password: ")?;

    let mut atlas = AtlasClient::connect(&endpoint)
        .await
        .with_context(|| format!("connect to atlas at '{server}'"))?;
    let (_, _, channel) =
        atlas_client::connect_to_capability(&mut atlas, CONSUMER_ID, LOGIN_CONTRACT)
            .await
            .context("locate Keystone login through Atlas")?;
    let response = RobonixSystemKeystoneLoginClient::new(channel)
        .login(Request::new(LoginRequest {
            username: username.trim().to_string(),
            password: password.clone(),
        }))
        .await
        .context("Keystone login failed")?
        .into_inner();
    let user = response
        .user
        .context("Keystone login response did not include a user")?;

    if user.password_change_required {
        change_required_password(&mut atlas, &response.session_token, &password).await?;
    }
    revoke_previous_session(&mut atlas, &endpoint, &response.session_token).await?;
    save_session(&StoredSession {
        atlas_endpoint: endpoint,
        session_token: response.session_token,
        expires_at_ms: response.expires_at_ms,
    })?;
    println!("Logged in as {}.", user.username);
    Ok(())
}

pub async fn logout(server: &str) -> Result<()> {
    let endpoint = super::canonical_endpoint(server);
    let environment_token = environment_session_token();
    let stored = match load_session() {
        Ok(stored) => stored.filter(|session| session.atlas_endpoint == endpoint),
        Err(error) if environment_token.is_some() => {
            eprintln!(
                "Ignoring unreadable saved session because ROBONIX_SESSION_TOKEN is set: {error:#}"
            );
            None
        }
        Err(error) => return Err(error),
    };
    let tokens = logout_tokens(environment_token, stored.as_ref());
    if tokens.is_empty() {
        anyhow::bail!("no login session for this Atlas endpoint");
    }

    let mut atlas = AtlasClient::connect(&endpoint)
        .await
        .with_context(|| format!("connect to atlas at '{server}'"))?;
    for token in tokens {
        logout_with_atlas(&mut atlas, &token).await?;
    }
    if stored.is_some() {
        remove_session()?;
    }
    println!("Logged out.");
    Ok(())
}

async fn logout_with_atlas(atlas: &mut AtlasClient, token: &str) -> Result<()> {
    let (_, _, channel) = atlas_client::connect_to_capability(atlas, CONSUMER_ID, LOGOUT_CONTRACT)
        .await
        .context("locate Keystone logout through Atlas")?;
    RobonixSystemKeystoneLogoutClient::new(channel)
        .logout(Request::new(LogoutRequest {
            session_token: token.to_string(),
        }))
        .await
        .context("Keystone logout failed")?;
    Ok(())
}

async fn revoke_previous_session(
    atlas: &mut AtlasClient,
    endpoint: &str,
    new_token: &str,
) -> Result<()> {
    let previous = match load_session() {
        Ok(previous) => previous.filter(|session| session.atlas_endpoint == endpoint),
        Err(error) => {
            eprintln!("Ignoring unreadable previous session while logging in: {error:#}");
            None
        }
    };
    if let Some(previous) = previous
        && previous.session_token != new_token
    {
        logout_with_atlas(atlas, &previous.session_token)
            .await
            .context("revoke previous saved Keystone session")?;
    }
    Ok(())
}

async fn has_keystone(atlas: &mut AtlasClient) -> Result<bool> {
    let providers = atlas
        .query_capabilities("", LOGIN_CONTRACT, atlas_pb::Transport::Grpc)
        .await
        .context("query Keystone login capability")?;
    Ok(providers.iter().any(|provider| {
        provider.capabilities.iter().any(|capability| {
            capability.contract_id == LOGIN_CONTRACT
                && capability.transport == atlas_pb::Transport::Grpc as i32
        })
    }))
}

async fn get_profile(atlas: &mut AtlasClient, token: &str) -> Result<User> {
    let (_, _, channel) =
        atlas_client::connect_to_capability(atlas, CONSUMER_ID, GET_PROFILE_CONTRACT)
            .await
            .context("locate Keystone profile through Atlas")?;
    RobonixSystemKeystoneGetProfileClient::new(channel)
        .get_profile(Request::new(GetProfileRequest {
            session_token: token.to_string(),
        }))
        .await
        .context("resolve Keystone login session")?
        .into_inner()
        .user
        .context("Keystone profile response did not include a user")
}

async fn change_required_password(
    atlas: &mut AtlasClient,
    session_token: &str,
    current_password: &str,
) -> Result<()> {
    eprintln!("Keystone requires a password change.");
    let new_password = prompt_password("New password: ")?;
    let confirmation = prompt_password("Confirm new password: ")?;
    if new_password != confirmation {
        anyhow::bail!("password confirmation does not match");
    }
    let (_, _, channel) =
        atlas_client::connect_to_capability(atlas, CONSUMER_ID, CHANGE_PASSWORD_CONTRACT)
            .await
            .context("locate Keystone password change through Atlas")?;
    RobonixSystemKeystoneChangePasswordClient::new(channel)
        .change_password(Request::new(ChangePasswordRequest {
            session_token: session_token.to_string(),
            current_password: current_password.to_string(),
            new_password,
        }))
        .await
        .context("Keystone password change failed")?;
    Ok(())
}

fn prompt_line(prompt: &str) -> Result<String> {
    eprint!("{prompt}");
    io::stderr().flush()?;
    let mut value = String::new();
    io::stdin().read_line(&mut value)?;
    let value = value.trim().to_string();
    if value.is_empty() {
        anyhow::bail!("username cannot be empty");
    }
    Ok(value)
}

fn prompt_password(prompt: &str) -> Result<String> {
    eprint!("{prompt}");
    io::stderr().flush()?;
    if !io::stdin().is_terminal() {
        let mut value = String::new();
        io::stdin().read_line(&mut value)?;
        return Ok(value.trim_end_matches(['\r', '\n']).to_string());
    }

    crossterm::terminal::enable_raw_mode()?;
    let _guard = RawModeGuard;
    let mut password = String::new();
    loop {
        if let Event::Key(key) = event::read()? {
            match key.code {
                KeyCode::Enter => {
                    eprintln!();
                    return Ok(password);
                }
                KeyCode::Backspace => {
                    password.pop();
                }
                KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                    eprintln!();
                    anyhow::bail!("login cancelled");
                }
                KeyCode::Char(character)
                    if !key
                        .modifiers
                        .intersects(KeyModifiers::CONTROL | KeyModifiers::ALT) =>
                {
                    password.push(character);
                }
                _ => {}
            }
        }
    }
}

struct RawModeGuard;

impl Drop for RawModeGuard {
    fn drop(&mut self) {
        let _ = crossterm::terminal::disable_raw_mode();
    }
}

fn session_path() -> Result<std::path::PathBuf> {
    Ok(Config::robonix_home_dir()?.join("session.yaml"))
}

fn load_session() -> Result<Option<StoredSession>> {
    load_session_from_path(&session_path()?)
}

fn load_session_from_path(path: &Path) -> Result<Option<StoredSession>> {
    match std::fs::symlink_metadata(path) {
        Ok(metadata) if metadata.file_type().is_symlink() => {
            anyhow::bail!(
                "login session '{}' must not be a symbolic link",
                path.display()
            );
        }
        Ok(metadata) if !metadata.is_file() => {
            anyhow::bail!("login session '{}' is not a regular file", path.display());
        }
        Ok(_) => secure_session_file(path)?,
        Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(None),
        Err(error) => {
            return Err(error)
                .with_context(|| format!("inspect login session '{}'", path.display()));
        }
    }
    match std::fs::read_to_string(path) {
        Ok(raw) => serde_yaml::from_str(&raw)
            .with_context(|| format!("parse login session '{}'", path.display()))
            .map(Some),
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(None),
        Err(error) => {
            Err(error).with_context(|| format!("read login session '{}'", path.display()))
        }
    }
}

fn secure_session_file(path: &Path) -> Result<()> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        if std::fs::metadata(path)?.permissions().mode() & 0o077 != 0 {
            std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))?;
        }
    }
    Ok(())
}

fn save_session(session: &StoredSession) -> Result<()> {
    let path = session_path()?;
    let parent = path.parent().context("login session path has no parent")?;
    std::fs::create_dir_all(parent)
        .with_context(|| format!("create login session directory '{}'", parent.display()))?;
    let temporary = parent.join(format!(".session-{}.tmp", uuid::Uuid::new_v4()));
    let result =
        write_private(&temporary, serde_yaml::to_string(session)?.as_bytes()).and_then(|()| {
            std::fs::rename(&temporary, &path)
                .with_context(|| format!("save login session '{}'", path.display()))
        });
    if result.is_err() {
        let _ = std::fs::remove_file(&temporary);
    }
    result
}

fn write_private(path: &Path, contents: &[u8]) -> Result<()> {
    let mut options = OpenOptions::new();
    options.write(true).create_new(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.mode(0o600);
    }
    let mut file = options
        .open(path)
        .with_context(|| format!("create login session '{}'", path.display()))?;
    file.write_all(contents)
        .with_context(|| format!("write login session '{}'", path.display()))?;
    file.sync_all()
        .with_context(|| format!("sync login session '{}'", path.display()))?;
    Ok(())
}

fn remove_session() -> Result<()> {
    let path = session_path()?;
    match std::fs::remove_file(&path) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(error) => {
            Err(error).with_context(|| format!("remove login session '{}'", path.display()))
        }
    }
}

fn session_is_expired(session: &StoredSession) -> bool {
    let now_ms = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64;
    session.expires_at_ms <= now_ms
}

fn local_username() -> String {
    std::env::var("USER")
        .or_else(|_| std::env::var("USERNAME"))
        .unwrap_or_else(|_| "user".to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stored_session_expires_at_its_deadline() {
        let session = StoredSession {
            atlas_endpoint: "http://127.0.0.1:50051".to_string(),
            session_token: "secret".to_string(),
            expires_at_ms: u64::MAX,
        };
        assert!(!session_is_expired(&session));

        let expired = StoredSession {
            expires_at_ms: 0,
            ..session
        };
        assert!(session_is_expired(&expired));
    }

    #[test]
    fn canonical_endpoint_normalizes_loopback() {
        assert_eq!(
            super::super::canonical_endpoint("localhost:50051"),
            "http://127.0.0.1:50051"
        );
        assert_eq!(
            super::super::canonical_endpoint("http://127.0.0.1:50051"),
            "http://127.0.0.1:50051"
        );
        assert_eq!(
            super::super::canonical_endpoint("http://localhost.example:50051"),
            "http://localhost.example:50051"
        );
    }

    #[test]
    fn only_unauthenticated_profile_errors_invalidate_a_saved_session() {
        let invalid =
            anyhow::Error::new(tonic::Status::unauthenticated("expired")).context("get profile");
        assert!(is_authentication_failure(&invalid));

        let unavailable =
            anyhow::Error::new(tonic::Status::unavailable("offline")).context("get profile");
        assert!(!is_authentication_failure(&unavailable));

        let password_change = anyhow::Error::new(tonic::Status::failed_precondition(
            "password change is required",
        ))
        .context("get profile");
        assert!(is_password_change_required(&password_change));
    }

    #[test]
    fn logout_revokes_environment_and_saved_tokens_without_duplicates() {
        let stored = StoredSession {
            atlas_endpoint: "http://127.0.0.1:50051".to_string(),
            session_token: "saved".to_string(),
            expires_at_ms: u64::MAX,
        };
        assert_eq!(
            logout_tokens(Some("environment".to_string()), Some(&stored)),
            ["environment".to_string(), "saved".to_string()]
        );
        assert_eq!(
            logout_tokens(Some("saved".to_string()), Some(&stored)),
            ["saved".to_string()]
        );
    }

    #[cfg(unix)]
    #[test]
    fn private_session_file_is_owner_only() {
        use std::os::unix::fs::PermissionsExt;

        let directory = std::env::temp_dir().join(format!("rbnx-auth-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir(&directory).expect("create test directory");
        let path = directory.join("session.yaml");

        write_private(&path, b"session_token: secret\n").expect("write private session");
        let mode = std::fs::metadata(&path)
            .expect("session metadata")
            .permissions()
            .mode()
            & 0o777;
        std::fs::remove_dir_all(&directory).expect("remove test directory");

        assert_eq!(mode, 0o600);
    }

    #[cfg(unix)]
    #[test]
    fn loading_repairs_permissions_and_rejects_symlinks() {
        use std::os::unix::fs::PermissionsExt;

        let directory = std::env::temp_dir().join(format!("rbnx-auth-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir(&directory).expect("create test directory");
        let path = directory.join("session.yaml");
        std::fs::write(
            &path,
            "atlas_endpoint: http://127.0.0.1:50051\nsession_token: secret\nexpires_at_ms: 1\n",
        )
        .expect("write session");
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o644))
            .expect("loosen session permissions");

        load_session_from_path(&path).expect("load session");
        let mode = std::fs::metadata(&path)
            .expect("session metadata")
            .permissions()
            .mode()
            & 0o777;
        assert_eq!(mode, 0o600);

        let link = directory.join("session-link.yaml");
        std::os::unix::fs::symlink(&path, &link).expect("create session symlink");
        let error = match load_session_from_path(&link) {
            Ok(_) => panic!("accepted session symlink"),
            Err(error) => error,
        };
        assert!(error.to_string().contains("symbolic link"));

        std::fs::remove_dir_all(&directory).expect("remove test directory");
    }
}
