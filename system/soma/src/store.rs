// SPDX-License-Identifier: MulanPSL-2.0
//
// One soma process serves one robot's body data. The v1 shape carried
// a `BTreeMap<robot_id, RobotRecord>` plus a `default_robot` selector;
// in practice every deployment we've ever run had a single entry and
// the reviewer flagged the map as speculative generality. This file
// now loads a single `SomaBody` from the `robot_yaml` handed to us by
// `SomaConfig` and exposes its YAML + URDF text directly to the gRPC
// service. If a future deployment needs multi-robot fan-out, that's a
// separate service on top, not a knob buried in soma.

use anyhow::{Context, Result, bail};
use serde::Deserialize;
use std::path::{Path, PathBuf};
use thiserror::Error;

#[derive(Debug, Clone)]
pub struct SomaBody {
    pub robot_id: String,
    pub yaml_path: PathBuf,
    pub yaml_text: String,
    pub urdf_path: PathBuf,
    pub urdf_xml: String,
}

#[derive(Debug, Error)]
pub enum StoreError {
    #[error("unknown Soma robot_id '{0}'")]
    NotFound(String),
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

impl SomaBody {
    /// Load raw Soma YAML at `yaml_path` and its referenced URDF.
    /// The URDF `path` inside the YAML is resolved relative to the
    /// YAML file's own directory when relative — matches how a human
    /// operator would read it.
    pub fn load(yaml_path: &Path) -> Result<Self> {
        let yaml_text = std::fs::read_to_string(yaml_path)
            .with_context(|| format!("read '{}'", yaml_path.display()))?;
        let doc: MinimalSomaDoc = serde_yaml::from_str(&yaml_text)
            .with_context(|| format!("parse '{}'", yaml_path.display()))?;
        if doc.robot.id.trim().is_empty() {
            bail!("robot.id must not be empty in '{}'", yaml_path.display());
        }
        let yaml_dir = yaml_path
            .parent()
            .with_context(|| format!("'{}' has no parent directory", yaml_path.display()))?;
        let urdf_path = if doc.urdf.path.is_absolute() {
            doc.urdf.path.clone()
        } else {
            yaml_dir.join(&doc.urdf.path)
        };
        let urdf_xml = std::fs::read_to_string(&urdf_path)
            .with_context(|| format!("read URDF '{}'", urdf_path.display()))?;
        Ok(Self {
            robot_id: doc.robot.id,
            yaml_path: yaml_path.to_path_buf(),
            yaml_text,
            urdf_path,
            urdf_xml,
        })
    }

    /// Return this body if `robot_id` is empty or matches the loaded
    /// robot; otherwise `NotFound`. The gRPC service keeps the
    /// `robot_id` parameter for wire compatibility with the v1 clients,
    /// but there's no routing to do anymore.
    pub fn resolve(&self, robot_id: &str) -> std::result::Result<&Self, StoreError> {
        let trimmed = robot_id.trim();
        if trimmed.is_empty() || trimmed == self.robot_id {
            Ok(self)
        } else {
            Err(StoreError::NotFound(trimmed.to_string()))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture_yaml() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join("examples/test_ci/soma.yaml")
    }

    #[test]
    fn loads_raw_yaml_and_urdf() {
        let body = SomaBody::load(&fixture_yaml()).expect("load body");
        assert_eq!(body.robot_id, "test_ci_robot");
        assert!(body.yaml_text.contains("robot:"));
        assert!(body.urdf_xml.contains("<robot name=\"test_ci_robot\">"));
    }

    #[test]
    fn empty_robot_id_resolves_to_the_loaded_body() {
        let body = SomaBody::load(&fixture_yaml()).expect("load body");
        let resolved = body.resolve("").expect("empty id resolves");
        assert_eq!(resolved.robot_id, "test_ci_robot");
    }

    #[test]
    fn mismatching_robot_id_is_not_found() {
        let body = SomaBody::load(&fixture_yaml()).expect("load body");
        let err = body.resolve("someone_else").expect_err("must not match");
        assert!(matches!(err, StoreError::NotFound(_)));
    }
}
