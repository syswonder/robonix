// SPDX-License-Identifier: MulanPSL-2.0

//! Internal Keystone client used by Liaison to authenticate task and voice sessions.
//!
//! Keystone is the only provider of account capabilities. Liaison deliberately
//! does not implement or re-export any Keystone server trait.

use std::sync::Arc;

use robonix_atlas::client::AtlasClient;
use tokio::sync::Mutex;
use tonic::Status;

use crate::pb::contracts::{
    robonix_system_keystone_get_profile_client::RobonixSystemKeystoneGetProfileClient,
    robonix_system_keystone_verify_voice_client::RobonixSystemKeystoneVerifyVoiceClient,
};
use crate::pb::keystone::{GetProfileRequest, User, VerifyVoiceRequest, VerifyVoiceResponse};
use crate::voice::resolve_endpoint;

const KEYSTONE_PROVIDER_ID: &str = "keystone";
const GET_PROFILE_CONTRACT: &str = "robonix/system/keystone/get_profile";
const VERIFY_VOICE_CONTRACT: &str = "robonix/system/keystone/verify_voice";

#[derive(Clone)]
pub struct KeystoneGateway {
    fallback_endpoint: String,
    atlas: Arc<Mutex<AtlasClient>>,
}

impl KeystoneGateway {
    pub fn new(endpoint: impl Into<String>, atlas: Arc<Mutex<AtlasClient>>) -> Self {
        Self {
            fallback_endpoint: normalize_endpoint(endpoint.into()),
            atlas,
        }
    }

    async fn endpoint(&self, contract_id: &str) -> String {
        resolve_endpoint(&self.atlas, contract_id, KEYSTONE_PROVIDER_ID)
            .await
            .unwrap_or_else(|| self.fallback_endpoint.clone())
    }

    /// Resolve an opaque login session to the canonical account.
    pub async fn resolve_session(&self, token: &str) -> Result<User, Status> {
        let endpoint = self.endpoint(GET_PROFILE_CONTRACT).await;
        let response = RobonixSystemKeystoneGetProfileClient::connect(endpoint.clone())
            .await
            .map_err(|error| {
                Status::unavailable(format!("connect Keystone at {endpoint}: {error}"))
            })?
            .get_profile(GetProfileRequest {
                session_token: token.to_string(),
            })
            .await?
            .into_inner();
        response
            .user
            .ok_or_else(|| Status::internal("Keystone returned an empty user profile"))
    }

    pub async fn client_verify_voice(
        &self,
        token: &str,
        external_subject_id: &str,
        confidence: f32,
        minimum_confidence: f32,
    ) -> Result<VerifyVoiceResponse, Status> {
        let endpoint = self.endpoint(VERIFY_VOICE_CONTRACT).await;
        RobonixSystemKeystoneVerifyVoiceClient::connect(endpoint.clone())
            .await
            .map_err(|error| {
                Status::unavailable(format!("connect Keystone at {endpoint}: {error}"))
            })?
            .verify_voice(VerifyVoiceRequest {
                session_token: token.to_string(),
                external_subject_id: external_subject_id.to_string(),
                confidence,
                minimum_confidence,
            })
            .await
            .map(|response| response.into_inner())
    }
}

fn normalize_endpoint(endpoint: String) -> String {
    if endpoint.starts_with("http://") || endpoint.starts_with("https://") {
        endpoint
    } else {
        format!("http://{endpoint}")
    }
}
