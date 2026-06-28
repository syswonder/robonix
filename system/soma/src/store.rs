// SPDX-License-Identifier: MulanPSL-2.0

use crate::config::SomaConfig;
use anyhow::{Context, Result, bail};
use serde::Deserialize;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use thiserror::Error;

#[derive(Debug, Clone)]
pub struct SomaStore {
    robots: BTreeMap<String, RobotRecord>,
    default_robot: Option<String>,
}

#[derive(Debug, Clone)]
pub struct RobotRecord {
    pub robot_id: String,
    pub package_path: PathBuf,
    pub yaml_path: PathBuf,
    pub yaml_text: String,
    pub urdf_path: PathBuf,
    pub urdf_xml: String,
}

#[derive(Debug, Error)]
pub enum StoreError {
    #[error("unknown Soma robot_id '{0}'")]
    NotFound(String),
    #[error("robot_id is empty and no default_robot is configured")]
    MissingDefaultRobot,
}

#[derive(Debug, Deserialize)]
struct MinimalSomaDoc {
    urdf: MinimalUrdf,
    robot: MinimalRobot,
}

#[derive(Debug, Deserialize)]
struct MinimalUrdf {
    path: PathBuf,
}

#[derive(Debug, Deserialize)]
struct MinimalRobot {
    id: String,
}

impl SomaStore {
    /// Load raw Soma YAML and URDF text for every configured deployment.
    pub fn load(config: &SomaConfig) -> Result<Self> {
        let mut robots = BTreeMap::new();
        for deployment in &config.deployments {
            let record =
                RobotRecord::load_from_deployment(&deployment.path).with_context(|| {
                    format!("load Soma body data from '{}'", deployment.path.display())
                })?;
            if robots.contains_key(&record.robot_id) {
                bail!("duplicate Soma robot id '{}'", record.robot_id);
            }
            robots.insert(record.robot_id.clone(), record);
        }
        if let Some(default) = &config.default_robot
            && !robots.contains_key(default)
        {
            bail!("default_robot '{default}' not found in loaded Soma robots");
        }
        Ok(Self {
            robots,
            default_robot: config.default_robot.clone(),
        })
    }

    /// Get a robot by id; empty id selects the configured default or the only robot.
    pub fn get(&self, robot_id: &str) -> std::result::Result<&RobotRecord, StoreError> {
        let trimmed = robot_id.trim();
        let id = if trimmed.is_empty() {
            self.resolve_default_robot()?
        } else {
            trimmed
        };
        self.robots
            .get(id)
            .ok_or_else(|| StoreError::NotFound(id.to_string()))
    }

    pub fn len(&self) -> usize {
        self.robots.len()
    }

    pub fn is_empty(&self) -> bool {
        self.robots.is_empty()
    }

    fn resolve_default_robot(&self) -> std::result::Result<&str, StoreError> {
        if let Some(id) = &self.default_robot {
            return Ok(id);
        }
        if self.robots.len() == 1 {
            return Ok(self
                .robots
                .keys()
                .next()
                .map(String::as_str)
                .expect("one robot is loaded"));
        }
        Err(StoreError::MissingDefaultRobot)
    }
}

impl RobotRecord {
    /// Read one deployment's raw Soma YAML, minimally parse robot id and URDF path, then cache URDF text.
    pub fn load_from_deployment(deployment_path: &Path) -> Result<Self> {
        let yaml_path = find_yaml_file(deployment_path)?;
        let yaml_text = std::fs::read_to_string(&yaml_path)
            .with_context(|| format!("read '{}'", yaml_path.display()))?;
        let doc: MinimalSomaDoc = serde_yaml::from_str(&yaml_text)
            .with_context(|| format!("parse '{}'", yaml_path.display()))?;
        if doc.robot.id.trim().is_empty() {
            bail!("robot.id must not be empty in '{}'", yaml_path.display());
        }
        let urdf_path = if doc.urdf.path.is_absolute() {
            doc.urdf.path
        } else {
            deployment_path.join(doc.urdf.path)
        };
        let urdf_xml = std::fs::read_to_string(&urdf_path)
            .with_context(|| format!("read URDF '{}'", urdf_path.display()))?;
        Ok(Self {
            robot_id: doc.robot.id,
            package_path: deployment_path.to_path_buf(),
            yaml_path,
            yaml_text,
            urdf_path,
            urdf_xml,
        })
    }
}

fn find_yaml_file(deployment_path: &Path) -> Result<PathBuf> {
    for name in ["soma.yaml", "robonix.soma.yaml"] {
        let path = deployment_path.join(name);
        if path.is_file() {
            return Ok(path);
        }
    }
    bail!(
        "no Soma YAML file found in '{}' (expected soma.yaml or compatible name)",
        deployment_path.display()
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{DeploymentConfig, SomaConfig};

    fn fixture_config() -> SomaConfig {
        let deployment = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join("examples/test_ci");
        SomaConfig {
            atlas_endpoint: "127.0.0.1:50051".into(),
            listen: "127.0.0.1:50091".into(),
            provider_id: "soma".into(),
            robonix_root: PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../.."),
            default_robot: Some("test_ci_robot".into()),
            deployments: vec![DeploymentConfig { path: deployment }],
            start_packages: false,
            rbnx_bin: "rbnx".into(),
        }
    }

    #[test]
    fn loads_raw_yaml_and_urdf() {
        let store = SomaStore::load(&fixture_config()).expect("load store");
        let robot = store.get("test_ci_robot").expect("robot");
        assert!(robot.yaml_text.contains("robot:"));
        assert!(robot.urdf_xml.contains("<robot name=\"test_ci_robot\">"));
        assert_eq!(store.len(), 1);
    }

    #[test]
    fn empty_robot_id_uses_default() {
        let store = SomaStore::load(&fixture_config()).expect("load store");
        let robot = store.get("").expect("default robot");
        assert_eq!(robot.robot_id, "test_ci_robot");
    }
}
