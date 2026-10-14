// SPDX-License-Identifier: MulanPSL-2.0

use std::sync::Arc;

use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use tokio::sync::Mutex;
use tonic::{Request, Response, Status};

use robonix_keystone::pb::contracts::{
    robonix_service_voiceprint_delete_client::RobonixServiceVoiceprintDeleteClient,
    robonix_service_voiceprint_enroll_client::RobonixServiceVoiceprintEnrollClient,
    robonix_system_keystone_admin_delete_user_server::RobonixSystemKeystoneAdminDeleteUser,
    robonix_system_keystone_admin_reset_voiceprint_server::RobonixSystemKeystoneAdminResetVoiceprint,
    robonix_system_keystone_admin_update_user_server::RobonixSystemKeystoneAdminUpdateUser,
    robonix_system_keystone_change_password_server::RobonixSystemKeystoneChangePassword,
    robonix_system_keystone_get_profile_server::RobonixSystemKeystoneGetProfile,
    robonix_system_keystone_get_system_config_server::RobonixSystemKeystoneGetSystemConfig,
    robonix_system_keystone_get_voiceprint_preview_server::RobonixSystemKeystoneGetVoiceprintPreview,
    robonix_system_keystone_list_users_server::RobonixSystemKeystoneListUsers,
    robonix_system_keystone_login_server::RobonixSystemKeystoneLogin,
    robonix_system_keystone_logout_server::RobonixSystemKeystoneLogout,
    robonix_system_keystone_register_server::RobonixSystemKeystoneRegister,
    robonix_system_keystone_replace_voiceprint_server::RobonixSystemKeystoneReplaceVoiceprint,
    robonix_system_keystone_unbind_voiceprint_server::RobonixSystemKeystoneUnbindVoiceprint,
    robonix_system_keystone_update_profile_server::RobonixSystemKeystoneUpdateProfile,
    robonix_system_keystone_update_system_config_server::RobonixSystemKeystoneUpdateSystemConfig,
    robonix_system_keystone_verify_voice_server::RobonixSystemKeystoneVerifyVoice,
};
use robonix_keystone::pb::keystone::{
    AdminDeleteUserRequest, AdminDeleteUserResponse, AdminResetVoiceprintRequest,
    AdminResetVoiceprintResponse, AdminUpdateUserRequest, AdminUpdateUserResponse,
    ChangePasswordRequest, ChangePasswordResponse, GetProfileRequest, GetProfileResponse,
    GetSystemConfigRequest, GetSystemConfigResponse, GetVoiceprintPreviewRequest,
    GetVoiceprintPreviewResponse, ListUsersRequest, ListUsersResponse, LoginRequest, LoginResponse,
    LogoutRequest, LogoutResponse, RegisterRequest, RegisterResponse, ReplaceVoiceprintRequest,
    ReplaceVoiceprintResponse, UnbindVoiceprintRequest, UnbindVoiceprintResponse,
    UpdateProfileRequest, UpdateProfileResponse, UpdateSystemConfigRequest,
    UpdateSystemConfigResponse, VerifyVoiceRequest, VerifyVoiceResponse,
};
use robonix_keystone::pb::voiceprint::{DeleteEnrolledRequest, EnrollRequest};
use robonix_keystone::{AuthSession, KeystoneError, KeystoneStore, User};

#[derive(Clone)]
pub struct KeystoneService {
    store: KeystoneStore,
    atlas: Arc<Mutex<AtlasClient>>,
}

impl KeystoneService {
    pub fn new(store: KeystoneStore, atlas: Arc<Mutex<AtlasClient>>) -> Self {
        Self { store, atlas }
    }

    async fn resolve_endpoint(
        &self,
        contract_id: &str,
        provider_id: &str,
    ) -> Result<String, Status> {
        let mut atlas = self.atlas.lock().await;
        let transport = atlas_pb::Transport::Grpc;
        let providers = atlas
            .query_capabilities("", contract_id, transport)
            .await
            .map_err(|error| Status::unavailable(format!("query Atlas: {error:#}")))?;
        let provider = if provider_id.is_empty() {
            providers.iter().find(|provider| {
                provider.capabilities.iter().any(|capability| {
                    capability.contract_id == contract_id
                        && capability.transport == transport as i32
                })
            })
        } else {
            providers
                .iter()
                .find(|provider| provider.id == provider_id || provider.namespace == provider_id)
        }
        .ok_or_else(|| Status::unavailable(format!("no provider for '{contract_id}'")))?;
        let (_, endpoint, _) = atlas
            .connect_capability("keystone", &provider.id, contract_id, transport)
            .await
            .map_err(|error| Status::unavailable(format!("connect capability: {error:#}")))?;
        Ok(normalize_endpoint(&endpoint))
    }

    async fn delete_external_voiceprint(
        &self,
        user_id: &str,
        provider_id: &str,
    ) -> Result<(), Status> {
        let endpoint = self
            .resolve_endpoint("robonix/service/voiceprint/delete", provider_id)
            .await?;
        let response = RobonixServiceVoiceprintDeleteClient::connect(endpoint.clone())
            .await
            .map_err(|error| {
                Status::unavailable(format!("connect Voiceprint at {endpoint}: {error}"))
            })?
            .delete_enrolled(DeleteEnrolledRequest {
                user_id: user_id.to_string(),
            })
            .await?
            .into_inner();
        if response.success {
            Ok(())
        } else {
            Err(Status::failed_precondition(format!(
                "Voiceprint deletion failed: {}",
                response.error
            )))
        }
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneRegister for KeystoneService {
    async fn register(
        &self,
        request: Request<RegisterRequest>,
    ) -> Result<Response<RegisterResponse>, Status> {
        let input = request.into_inner();
        self.store
            .register(
                &input.username,
                &input.display_name,
                &input.email,
                &input.password,
            )
            .map(auth_register_response)
            .map(Response::new)
            .map_err(status)
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneLogin for KeystoneService {
    async fn login(
        &self,
        request: Request<LoginRequest>,
    ) -> Result<Response<LoginResponse>, Status> {
        let input = request.into_inner();
        self.store
            .login(&input.username, &input.password)
            .map(auth_login_response)
            .map(Response::new)
            .map_err(status)
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneLogout for KeystoneService {
    async fn logout(
        &self,
        request: Request<LogoutRequest>,
    ) -> Result<Response<LogoutResponse>, Status> {
        self.store
            .logout(&request.into_inner().session_token)
            .map(|()| Response::new(LogoutResponse {}))
            .map_err(status)
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneGetProfile for KeystoneService {
    async fn get_profile(
        &self,
        request: Request<GetProfileRequest>,
    ) -> Result<Response<GetProfileResponse>, Status> {
        self.store
            .authenticate(&request.into_inner().session_token)
            .map(|user| GetProfileResponse {
                user: Some(user_message(user)),
            })
            .map(Response::new)
            .map_err(status)
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneUpdateProfile for KeystoneService {
    async fn update_profile(
        &self,
        request: Request<UpdateProfileRequest>,
    ) -> Result<Response<UpdateProfileResponse>, Status> {
        let input = request.into_inner();
        self.store
            .update_profile(&input.session_token, &input.display_name, &input.email)
            .map(|user| UpdateProfileResponse {
                user: Some(user_message(user)),
            })
            .map(Response::new)
            .map_err(status)
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneChangePassword for KeystoneService {
    async fn change_password(
        &self,
        request: Request<ChangePasswordRequest>,
    ) -> Result<Response<ChangePasswordResponse>, Status> {
        let input = request.into_inner();
        self.store
            .change_password(
                &input.session_token,
                &input.current_password,
                &input.new_password,
            )
            .map(|()| Response::new(ChangePasswordResponse {}))
            .map_err(status)
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneListUsers for KeystoneService {
    async fn list_users(
        &self,
        request: Request<ListUsersRequest>,
    ) -> Result<Response<ListUsersResponse>, Status> {
        self.store
            .list_users(&request.into_inner().session_token)
            .map(|users| ListUsersResponse {
                users: users.into_iter().map(user_message).collect(),
            })
            .map(Response::new)
            .map_err(status)
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneAdminUpdateUser for KeystoneService {
    async fn admin_update_user(
        &self,
        request: Request<AdminUpdateUserRequest>,
    ) -> Result<Response<AdminUpdateUserResponse>, Status> {
        let input = request.into_inner();
        self.store
            .admin_update_user(
                &input.session_token,
                &input.target_user_id,
                input.enabled,
                &input.roles,
                input.voice_guard_enabled,
            )
            .map(|user| AdminUpdateUserResponse {
                user: Some(user_message(user)),
            })
            .map(Response::new)
            .map_err(status)
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneAdminDeleteUser for KeystoneService {
    async fn admin_delete_user(
        &self,
        request: Request<AdminDeleteUserRequest>,
    ) -> Result<Response<AdminDeleteUserResponse>, Status> {
        let input = request.into_inner();
        let target = self
            .store
            .list_users(&input.session_token)
            .map_err(status)?
            .into_iter()
            .find(|user| user.user_id == input.target_user_id)
            .ok_or_else(|| Status::not_found("target user does not exist"))?;
        if target.voiceprint_enrolled {
            self.delete_external_voiceprint(&target.user_id, "").await?;
        }
        self.store
            .admin_delete_user(&input.session_token, &input.target_user_id)
            .map(|()| Response::new(AdminDeleteUserResponse {}))
            .map_err(status)
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneReplaceVoiceprint for KeystoneService {
    async fn replace_voiceprint(
        &self,
        request: Request<ReplaceVoiceprintRequest>,
    ) -> Result<Response<ReplaceVoiceprintResponse>, Status> {
        let input = request.into_inner();
        if input.audio_data.len() < 3_200 {
            return Err(Status::invalid_argument(
                "voiceprint sample must contain at least 0.1 seconds of 16 kHz PCM",
            ));
        }
        if input.audio_data.len() > 1_920_000 {
            return Err(Status::invalid_argument(
                "voiceprint sample must not exceed 60 seconds of 16 kHz PCM",
            ));
        }
        if !input.audio_data.len().is_multiple_of(2) {
            return Err(Status::invalid_argument(
                "voiceprint sample must contain complete signed 16-bit PCM frames",
            ));
        }
        let sample_rate_hz = if input.sample_rate_hz == 0 {
            16_000
        } else {
            input.sample_rate_hz
        };
        if !(8_000..=96_000).contains(&sample_rate_hz) {
            return Err(Status::invalid_argument(
                "voiceprint sample rate must be between 8000 and 96000 Hz",
            ));
        }
        let user = self
            .store
            .authenticate(&input.session_token)
            .map_err(status)?;
        if user.voiceprint_enrolled {
            self.delete_external_voiceprint(&user.user_id, &input.voiceprint_provider_id)
                .await?;
        }
        let endpoint = self
            .resolve_endpoint(
                "robonix/service/voiceprint/enroll",
                &input.voiceprint_provider_id,
            )
            .await?;
        let enrollment = RobonixServiceVoiceprintEnrollClient::connect(endpoint.clone())
            .await
            .map_err(|error| {
                Status::unavailable(format!("connect Voiceprint at {endpoint}: {error}"))
            })?
            .enroll(EnrollRequest {
                user_id: user.user_id.clone(),
                user_name: user.display_name,
                audio_data: input.audio_data.clone(),
                encoding: "pcm_s16le".to_string(),
                sample_rate_hz,
            })
            .await?
            .into_inner();
        if !enrollment.success {
            return Err(Status::failed_precondition(format!(
                "Voiceprint enrollment failed: {}",
                enrollment.error
            )));
        }
        match self.store.replace_voiceprint(
            &input.session_token,
            &user.user_id,
            &input.audio_data,
            sample_rate_hz,
        ) {
            Ok(user) => Ok(Response::new(ReplaceVoiceprintResponse {
                user: Some(user_message(user)),
            })),
            Err(error) => {
                let _ = self
                    .delete_external_voiceprint(&user.user_id, &input.voiceprint_provider_id)
                    .await;
                Err(status(error))
            }
        }
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneGetVoiceprintPreview for KeystoneService {
    async fn get_voiceprint_preview(
        &self,
        request: Request<GetVoiceprintPreviewRequest>,
    ) -> Result<Response<GetVoiceprintPreviewResponse>, Status> {
        let input = request.into_inner();
        let preview = self
            .store
            .voiceprint_preview(&input.session_token)
            .map_err(status)?;
        Ok(Response::new(match preview {
            Some(preview) => GetVoiceprintPreviewResponse {
                available: true,
                audio_data: preview.audio_data,
                sample_rate_hz: preview.sample_rate_hz,
                updated_at_ms: preview.updated_at_ms,
            },
            None => GetVoiceprintPreviewResponse {
                available: false,
                audio_data: Vec::new(),
                sample_rate_hz: 0,
                updated_at_ms: 0,
            },
        }))
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneUnbindVoiceprint for KeystoneService {
    async fn unbind_voiceprint(
        &self,
        request: Request<UnbindVoiceprintRequest>,
    ) -> Result<Response<UnbindVoiceprintResponse>, Status> {
        let input = request.into_inner();
        let user = self
            .store
            .authenticate(&input.session_token)
            .map_err(status)?;
        if user.voiceprint_enrolled {
            self.delete_external_voiceprint(&user.user_id, "").await?;
        }
        self.store
            .unbind_voiceprint(&input.session_token)
            .map(|user| UnbindVoiceprintResponse {
                user: Some(user_message(user)),
            })
            .map(Response::new)
            .map_err(status)
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneAdminResetVoiceprint for KeystoneService {
    async fn admin_reset_voiceprint(
        &self,
        request: Request<AdminResetVoiceprintRequest>,
    ) -> Result<Response<AdminResetVoiceprintResponse>, Status> {
        let input = request.into_inner();
        let target = self
            .store
            .list_users(&input.session_token)
            .map_err(status)?
            .into_iter()
            .find(|user| user.user_id == input.target_user_id)
            .ok_or_else(|| Status::not_found("target user does not exist"))?;
        if target.voiceprint_enrolled {
            self.delete_external_voiceprint(&target.user_id, "").await?;
        }
        self.store
            .admin_reset_voiceprint(&input.session_token, &input.target_user_id)
            .map(|user| AdminResetVoiceprintResponse {
                user: Some(user_message(user)),
            })
            .map(Response::new)
            .map_err(status)
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneVerifyVoice for KeystoneService {
    async fn verify_voice(
        &self,
        request: Request<VerifyVoiceRequest>,
    ) -> Result<Response<VerifyVoiceResponse>, Status> {
        let input = request.into_inner();
        self.store
            .verify_voice(
                &input.session_token,
                &input.external_subject_id,
                input.confidence,
                input.minimum_confidence,
            )
            .map(|(user, verified)| VerifyVoiceResponse {
                user: Some(user_message(user)),
                verified,
            })
            .map(Response::new)
            .map_err(status)
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneGetSystemConfig for KeystoneService {
    async fn get_system_config(
        &self,
        request: Request<GetSystemConfigRequest>,
    ) -> Result<Response<GetSystemConfigResponse>, Status> {
        self.store
            .authenticate(&request.into_inner().session_token)
            .and_then(|_| self.store.signup_enabled())
            .map(|signup_enabled| GetSystemConfigResponse { signup_enabled })
            .map(Response::new)
            .map_err(status)
    }
}

#[tonic::async_trait]
impl RobonixSystemKeystoneUpdateSystemConfig for KeystoneService {
    async fn update_system_config(
        &self,
        request: Request<UpdateSystemConfigRequest>,
    ) -> Result<Response<UpdateSystemConfigResponse>, Status> {
        let input = request.into_inner();
        self.store
            .set_signup_enabled(&input.session_token, input.signup_enabled)
            .map(|signup_enabled| UpdateSystemConfigResponse { signup_enabled })
            .map(Response::new)
            .map_err(status)
    }
}

fn auth_register_response(session: AuthSession) -> RegisterResponse {
    RegisterResponse {
        session_token: session.token,
        expires_at_ms: session.expires_at_ms,
        user: Some(user_message(session.user)),
    }
}

fn auth_login_response(session: AuthSession) -> LoginResponse {
    LoginResponse {
        session_token: session.token,
        expires_at_ms: session.expires_at_ms,
        user: Some(user_message(session.user)),
    }
}

fn user_message(user: User) -> robonix_keystone::pb::keystone::User {
    robonix_keystone::pb::keystone::User {
        user_id: user.user_id,
        username: user.username,
        display_name: user.display_name,
        email: user.email,
        enabled: user.enabled,
        roles: user.roles,
        voice_guard_enabled: user.voice_guard_enabled,
        voiceprint_enrolled: user.voiceprint_enrolled,
        password_change_required: user.password_change_required,
        created_at_ms: user.created_at_ms,
        updated_at_ms: user.updated_at_ms,
    }
}

fn status(error: KeystoneError) -> Status {
    match error {
        KeystoneError::AuthenticationFailed
        | KeystoneError::UserDisabled
        | KeystoneError::VoiceprintMismatch
        | KeystoneError::VoiceprintConfidenceLow => Status::unauthenticated(error.to_string()),
        KeystoneError::AdminRequired | KeystoneError::LastAdmin => {
            Status::permission_denied(error.to_string())
        }
        KeystoneError::UserNotFound(_) => Status::not_found(error.to_string()),
        KeystoneError::UsernameExists | KeystoneError::VoiceprintAlreadyBound => {
            Status::already_exists(error.to_string())
        }
        KeystoneError::SignupDisabled => Status::failed_precondition(error.to_string()),
        KeystoneError::InvalidUsername
        | KeystoneError::InvalidDisplayName
        | KeystoneError::InvalidEmail
        | KeystoneError::WeakPassword
        | KeystoneError::InvalidRole(_)
        | KeystoneError::InvalidVoiceprintSubject => Status::invalid_argument(error.to_string()),
        KeystoneError::PasswordHash(_)
        | KeystoneError::LockPoisoned
        | KeystoneError::Database(_) => Status::internal(error.to_string()),
    }
}

fn normalize_endpoint(endpoint: &str) -> String {
    let endpoint = endpoint.replace("localhost", "127.0.0.1");
    if endpoint.starts_with("http://") || endpoint.starts_with("https://") {
        endpoint
    } else {
        format!("http://{endpoint}")
    }
}
