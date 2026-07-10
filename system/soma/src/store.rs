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
    pub footprint: Option<Footprint>,
}

#[derive(Debug, Clone)]
pub struct Footprint {
    pub base_frame: String,
    pub points: Vec<FootprintPoint>,
    pub inscribed_radius_m: f64,
    pub circumscribed_radius_m: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct FootprintPoint {
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Error)]
pub enum StoreError {
    #[error("unknown Soma robot_id '{0}'")]
    NotFound(String),
    #[error("Soma robot '{0}' does not define robot.footprint")]
    MissingFootprint(String),
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
    #[serde(default)]
    footprint: Option<FootprintDoc>,
}

#[derive(Debug, Deserialize)]
struct FootprintDoc {
    base_frame: String,
    points: Vec<[f64; 2]>,
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
        let footprint = doc.robot.footprint.map(Footprint::from_doc).transpose()?;
        Ok(Self {
            robot_id: doc.robot.id,
            yaml_path: yaml_path.to_path_buf(),
            yaml_text,
            urdf_path,
            urdf_xml,
            footprint,
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

    pub fn footprint(&self) -> std::result::Result<&Footprint, StoreError> {
        self.footprint
            .as_ref()
            .ok_or_else(|| StoreError::MissingFootprint(self.robot_id.clone()))
    }
}

impl Footprint {
    fn from_doc(doc: FootprintDoc) -> Result<Self> {
        if doc.base_frame.trim().is_empty() {
            bail!("robot.footprint.base_frame must not be empty");
        }
        if doc.points.len() < 3 {
            bail!("robot.footprint.points must contain at least three vertices");
        }
        let points: Vec<FootprintPoint> = doc
            .points
            .into_iter()
            .map(|[x, y]| FootprintPoint { x, y })
            .collect();
        if points.iter().any(|p| !p.x.is_finite() || !p.y.is_finite()) {
            bail!("robot.footprint.points must be finite x/y pairs");
        }

        let circumscribed_radius_m = points
            .iter()
            .map(|p| p.x.hypot(p.y))
            .fold(0.0_f64, f64::max);
        let inscribed_radius_m = points
            .iter()
            .zip(points.iter().cycle().skip(1))
            .take(points.len())
            .map(|(a, b)| distance_from_origin_to_segment(*a, *b))
            .fold(f64::INFINITY, f64::min);
        if !inscribed_radius_m.is_finite() || inscribed_radius_m <= 0.0 {
            bail!("robot.footprint must enclose the base-frame origin");
        }

        Ok(Self {
            base_frame: doc.base_frame,
            points,
            inscribed_radius_m,
            circumscribed_radius_m,
        })
    }
}

fn distance_from_origin_to_segment(a: FootprintPoint, b: FootprintPoint) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let length_sq = dx * dx + dy * dy;
    if length_sq == 0.0 {
        return a.x.hypot(a.y);
    }
    let t = (-(a.x * dx + a.y * dy) / length_sq).clamp(0.0, 1.0);
    (a.x + t * dx).hypot(a.y + t * dy)
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
        let footprint = body.footprint().expect("fixture footprint");
        assert_eq!(footprint.base_frame, "base_link");
        assert_eq!(footprint.points.len(), 4);
        assert!((footprint.inscribed_radius_m - 0.1).abs() < 1e-9);
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
