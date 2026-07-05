// SPDX-License-Identifier: MulanPSL-2.0
//
// Pilot-side native Soma awareness.
//
// Soma exposes robot body data as gRPC contracts. Pilot fetches the
// default robot's Soma YAML and URDF once at startup, then injects the
// result into every turn's system prompt so the model knows its body
// without the user first calling a bridge tool.

use crate::pb::contracts::{
    robonix_system_soma_get_urdf_client::RobonixSystemSomaGetUrdfClient,
    robonix_system_soma_get_yaml_client::RobonixSystemSomaGetYamlClient,
};
use crate::pb::soma::{GetUrdfRequest, GetYamlRequest};
use anyhow::{Context, Result};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_scribe::warn;

const GET_YAML_CONTRACT: &str = "robonix/system/soma/get_yaml";
const GET_URDF_CONTRACT: &str = "robonix/system/soma/get_urdf";

pub async fn fetch_system_prompt_block(
    atlas: &mut AtlasClient,
    consumer_id: &str,
) -> Result<Option<String>> {
    let yaml = match fetch_yaml(atlas, consumer_id).await {
        Ok(text) => text,
        Err(e) => {
            warn!("[pilot/soma] get_yaml unavailable; continuing without Soma context: {e:#}");
            return Ok(None);
        }
    };
    let urdf = match fetch_urdf(atlas, consumer_id).await {
        Ok(text) => text,
        Err(e) => {
            warn!("[pilot/soma] get_urdf unavailable; continuing with YAML only: {e:#}");
            String::new()
        }
    };

    let mut block = String::from(
        "\n\n## Robot Body Context (from Soma)\n\n\
         This is the robot's self-description, loaded automatically at Pilot startup. \
         Treat it as authoritative HARD CONSTRAINTS for the robot's body, sensors, \
         frames, limits, and deployment-specific notes. Do not ask the user to call \
         Soma manually unless this context is absent or stale.\n\n\
         ### Hard planning rules from Soma\n\n\
         - Sensor placement and modality in `soma.yaml` are binding. Do not invent \
         sensors, viewpoints, arms, grippers, or degrees of freedom that are not listed.\n\
         - Before planning an observation, match the user's requested viewpoint \
         (front / rear / left / right / top, etc.) against the listed sensors' \
         `placement`, `human_label`, and `cannot_do` notes.\n\
         - If the requested viewpoint is not directly available from the sensors \
         listed in Soma, say so explicitly. Do NOT call a camera with one placement \
         and describe its image as if it came from a different placement.\n\
         - If a viewpoint can be achieved only by moving the base (for example, \
         rotate 180 degrees, then use the front camera), state that plan clearly \
         and use motion + observation capabilities rather than pretending a missing \
         sensor exists.\n\n\
         ### soma.yaml\n\n```yaml\n",
    );
    block.push_str(yaml.trim());
    block.push_str("\n```\n");
    if !urdf.trim().is_empty() {
        block.push_str("\n### URDF\n\n```xml\n");
        block.push_str(urdf.trim());
        block.push_str("\n```\n");
    }
    Ok(Some(block))
}

async fn fetch_yaml(atlas: &mut AtlasClient, consumer_id: &str) -> Result<String> {
    let (channel_id, _provider_id, channel) =
        atlas_client::connect_to_capability(atlas, consumer_id, GET_YAML_CONTRACT)
            .await
            .context("connect to Soma get_yaml")?;
    let result = async {
        let mut client = RobonixSystemSomaGetYamlClient::new(channel);
        let response = client
            .get_yaml(GetYamlRequest {
                robot_id: String::new(),
            })
            .await
            .context("call Soma get_yaml")?
            .into_inner();
        Ok::<_, anyhow::Error>(response.yaml_text)
    }
    .await;
    let _ = atlas.disconnect_capability(&channel_id).await;
    result
}

async fn fetch_urdf(atlas: &mut AtlasClient, consumer_id: &str) -> Result<String> {
    let (channel_id, _provider_id, channel) =
        atlas_client::connect_to_capability(atlas, consumer_id, GET_URDF_CONTRACT)
            .await
            .context("connect to Soma get_urdf")?;
    let result = async {
        let mut client = RobonixSystemSomaGetUrdfClient::new(channel);
        let response = client
            .get_urdf(GetUrdfRequest {
                robot_id: String::new(),
            })
            .await
            .context("call Soma get_urdf")?
            .into_inner();
        Ok::<_, anyhow::Error>(response.urdf_xml)
    }
    .await;
    let _ = atlas.disconnect_capability(&channel_id).await;
    result
}
