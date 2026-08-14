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
use std::collections::BTreeSet;
use std::path::{Component, Path, PathBuf};
use thiserror::Error;

#[derive(Debug, Clone)]
pub struct SomaBody {
    pub robot_id: String,
    pub display_name: String,
    pub model_name: String,
    pub root_link: String,
    pub components: Vec<SomaComponent>,
    pub yaml_path: PathBuf,
    pub yaml_text: String,
    pub urdf_path: PathBuf,
    pub urdf_xml: String,
    urdf_asset_paths: Vec<UrdfAssetPath>,
    pub footprint: Option<Footprint>,
    pub grippers: Vec<GripperConfig>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SomaComponent {
    pub id: String,
    pub parent_id: String,
    pub component_type: String,
    pub frame_id: String,
}

#[derive(Debug, Clone)]
struct UrdfAssetPath {
    wire_path: String,
    source_path: PathBuf,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SomaUrdfAsset {
    pub path: String,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct GripperConfig {
    pub component_id: String,
    pub provider_id: String,
    pub joint_name: String,
    pub open_position_m: f64,
    pub open_tolerance_m: f64,
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
    #[serde(default)]
    root_link: String,
    #[serde(default)]
    model_name: String,
}

#[derive(Debug, Deserialize)]
struct MinimalRobot {
    id: String,
    #[serde(default)]
    display_name: String,
    #[serde(default)]
    footprint: Option<FootprintDoc>,
    #[serde(default)]
    components: Vec<ComponentDoc>,
}

#[derive(Debug, Deserialize)]
struct ComponentDoc {
    id: String,
    #[serde(rename = "type")]
    component_type: String,
    #[serde(default)]
    urdf_link: String,
    #[serde(default)]
    urdf_joint: String,
    #[serde(default)]
    components: Vec<ComponentDoc>,
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
        let urdf_asset_paths = collect_urdf_asset_paths(&urdf_path, &urdf_xml)?;
        let footprint = doc.robot.footprint.map(Footprint::from_doc).transpose()?;
        let mut components = Vec::new();
        flatten_components(&doc.robot.components, "body", &mut components)?;
        let display_name = non_empty_or(&doc.robot.display_name, &doc.robot.id);
        let model_name = non_empty_or(&doc.urdf.model_name, &doc.robot.id);
        let root_link = non_empty_or(&doc.urdf.root_link, "base_link");
        let yaml_value: serde_yaml::Value = serde_yaml::from_str(&yaml_text)
            .with_context(|| format!("parse runtime state config in '{}'", yaml_path.display()))?;
        let mut grippers = Vec::new();
        collect_grippers(&yaml_value, None, &mut grippers);
        Ok(Self {
            robot_id: doc.robot.id,
            display_name,
            model_name,
            root_link,
            components,
            yaml_path: yaml_path.to_path_buf(),
            yaml_text,
            urdf_path,
            urdf_xml,
            urdf_asset_paths,
            footprint,
            grippers,
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

    /// Read requested URDF-local resources after validating their resolved paths.
    pub fn read_urdf_assets(&self) -> Result<Vec<SomaUrdfAsset>> {
        let urdf_dir = self.urdf_path.parent().with_context(|| {
            format!(
                "URDF '{}' has no parent directory",
                self.urdf_path.display()
            )
        })?;
        let canonical_dir = urdf_dir
            .canonicalize()
            .with_context(|| format!("resolve URDF directory '{}'", urdf_dir.display()))?;
        self.urdf_asset_paths
            .iter()
            .map(|asset| {
                let canonical_path = asset.source_path.canonicalize().with_context(|| {
                    format!("resolve URDF asset '{}'", asset.source_path.display())
                })?;
                if !canonical_path.starts_with(&canonical_dir) {
                    bail!(
                        "URDF asset '{}' resolves outside '{}'",
                        asset.wire_path,
                        urdf_dir.display()
                    );
                }
                let data = std::fs::read(&canonical_path)
                    .with_context(|| format!("read URDF asset '{}'", canonical_path.display()))?;
                Ok(SomaUrdfAsset {
                    path: asset.wire_path.clone(),
                    data,
                })
            })
            .collect()
    }
}

/// Index URDF-local mesh and texture paths without requiring files at startup.
fn collect_urdf_asset_paths(urdf_path: &Path, urdf_xml: &str) -> Result<Vec<UrdfAssetPath>> {
    let document = roxmltree::Document::parse(urdf_xml)
        .with_context(|| format!("parse URDF '{}'", urdf_path.display()))?;
    let urdf_dir = urdf_path
        .parent()
        .with_context(|| format!("URDF '{}' has no parent directory", urdf_path.display()))?;
    let mut paths = BTreeSet::new();
    for node in document.descendants().filter(|node| {
        node.is_element() && (node.has_tag_name("mesh") || node.has_tag_name("texture"))
    }) {
        let Some(filename) = node.attribute("filename") else {
            continue;
        };
        if let Some(path) = local_asset_path(filename)? {
            paths.insert(path);
        }
    }
    Ok(paths
        .into_iter()
        .map(|path| UrdfAssetPath {
            wire_path: path_for_wire(&path),
            source_path: urdf_dir.join(path),
        })
        .collect())
}

/// Return a normalized URDF-local path, or None for externally resolved URIs.
fn local_asset_path(filename: &str) -> Result<Option<PathBuf>> {
    let filename = filename.trim();
    if filename.is_empty()
        || filename.starts_with('/')
        || filename.contains("://")
        || filename.starts_with("data:")
    {
        return Ok(None);
    }
    if filename.contains('\\') {
        bail!("URDF asset paths must use POSIX separators: '{filename}'");
    }
    let path = Path::new(filename);
    if path
        .components()
        .any(|component| !matches!(component, Component::Normal(_) | Component::CurDir))
    {
        bail!("URDF asset path must stay below the URDF directory: '{filename}'");
    }
    Ok(Some(path.components().collect()))
}

/// Encode a validated native relative path as a browser-facing POSIX path.
fn path_for_wire(path: &Path) -> String {
    path.components()
        .filter_map(|component| match component {
            Component::Normal(value) => value.to_str(),
            _ => None,
        })
        .collect::<Vec<_>>()
        .join("/")
}

/// Flatten the recursive Soma component tree into stable `body/...` paths.
fn flatten_components(
    docs: &[ComponentDoc],
    parent_id: &str,
    output: &mut Vec<SomaComponent>,
) -> Result<()> {
    for doc in docs {
        if doc.id.trim().is_empty() {
            bail!("robot.components[].id must not be empty");
        }
        if doc.component_type.trim().is_empty() {
            bail!("robot component '{}' must define type", doc.id);
        }
        let id = format!("{parent_id}/{}", doc.id);
        let frame_id = non_empty_or(&doc.urdf_link, &doc.urdf_joint);
        output.push(SomaComponent {
            id: id.clone(),
            parent_id: parent_id.to_string(),
            component_type: doc.component_type.clone(),
            frame_id,
        });
        flatten_components(&doc.components, &id, output)?;
    }
    Ok(())
}

fn non_empty_or(value: &str, fallback: &str) -> String {
    if value.trim().is_empty() {
        fallback.to_string()
    } else {
        value.to_string()
    }
}

fn collect_grippers(
    value: &serde_yaml::Value,
    inherited_joint_provider: Option<&str>,
    out: &mut Vec<GripperConfig>,
) {
    let Some(map) = value.as_mapping() else {
        if let Some(items) = value.as_sequence() {
            for item in items {
                collect_grippers(item, inherited_joint_provider, out);
            }
        }
        return;
    };

    let provider =
        joint_state_provider(map).or_else(|| inherited_joint_provider.map(str::to_string));
    let component_type = yaml_str(map, "type")
        .or_else(|| yaml_str(map, "part_type"))
        .unwrap_or_default();
    if matches!(
        component_type.as_str(),
        "parallel_jaw_gripper" | "end_effector"
    ) {
        let state = map
            .get(serde_yaml::Value::String("state".into()))
            .and_then(serde_yaml::Value::as_mapping);
        let open_position_m = state.and_then(|m| yaml_f64(m, "open_position_m"));
        if let (Some(provider_id), Some(open_position_m)) = (provider.clone(), open_position_m) {
            out.push(GripperConfig {
                component_id: yaml_str(map, "id").unwrap_or_else(|| "gripper".into()),
                provider_id,
                joint_name: state
                    .and_then(|m| yaml_str(m, "joint_name"))
                    .unwrap_or_else(|| "gripper".into()),
                open_position_m,
                open_tolerance_m: state
                    .and_then(|m| yaml_f64(m, "open_tolerance_m"))
                    .unwrap_or(0.002),
            });
        }
    }

    for child_key in ["robot", "tree", "components", "children"] {
        if let Some(child) = map.get(serde_yaml::Value::String(child_key.into())) {
            collect_grippers(child, provider.as_deref(), out);
        }
    }
}

fn joint_state_provider(map: &serde_yaml::Mapping) -> Option<String> {
    for key in ["exports", "provided_by"] {
        let Some(rows) = map
            .get(serde_yaml::Value::String(key.into()))
            .and_then(serde_yaml::Value::as_sequence)
        else {
            continue;
        };
        for row in rows {
            let Some(row) = row.as_mapping() else {
                continue;
            };
            let direct_contract = yaml_str(row, "contract");
            if direct_contract.as_deref() == Some("robonix/primitive/arm/joint_states") {
                return yaml_str(row, "provider_id");
            }
            let provider_id = yaml_str(row, "provider_id");
            let has_contract = row
                .get(serde_yaml::Value::String("capabilities".into()))
                .and_then(serde_yaml::Value::as_sequence)
                .is_some_and(|caps| {
                    caps.iter().any(|cap| {
                        cap.as_mapping()
                            .and_then(|m| yaml_str(m, "path"))
                            .as_deref()
                            == Some("robonix/primitive/arm/joint_states")
                    })
                });
            if has_contract && provider_id.is_some() {
                return provider_id;
            }
        }
    }
    None
}

fn yaml_str(map: &serde_yaml::Mapping, key: &str) -> Option<String> {
    map.get(serde_yaml::Value::String(key.into()))
        .and_then(serde_yaml::Value::as_str)
        .map(str::to_string)
}

fn yaml_f64(map: &serde_yaml::Mapping, key: &str) -> Option<f64> {
    map.get(serde_yaml::Value::String(key.into()))
        .and_then(serde_yaml::Value::as_f64)
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

    #[test]
    fn reads_calibrated_gripper_state_config() {
        let value: serde_yaml::Value = serde_yaml::from_str(
            r#"
robot:
  components:
    - id: arm
      exports:
        - provider_id: piper_ctl
          capabilities:
            - { path: robonix/primitive/arm/joint_states }
      components:
        - id: gripper
          type: parallel_jaw_gripper
          state: { joint_name: gripper, open_position_m: 0.06937, open_tolerance_m: 0.002 }
"#,
        )
        .unwrap();
        let mut grippers = Vec::new();
        collect_grippers(&value, None, &mut grippers);
        assert_eq!(grippers.len(), 1);
        assert_eq!(grippers[0].provider_id, "piper_ctl");
        assert_eq!(grippers[0].joint_name, "gripper");
        assert!((grippers[0].open_position_m - 0.06937).abs() < 1e-9);
    }

    /// Missing Mesh files do not block startup but fail an explicit asset request.
    #[test]
    fn defers_urdf_asset_reads_until_requested() {
        let directory = tempfile::tempdir().expect("temp directory");
        let yaml_path = directory.path().join("soma.yaml");
        std::fs::write(
            &yaml_path,
            "urdf:\n  path: robot.urdf\nrobot:\n  id: lazy_assets\n",
        )
        .expect("write Soma YAML");
        std::fs::write(
            directory.path().join("robot.urdf"),
            r#"<robot name="lazy_assets"><link name="base_link"><visual><geometry><mesh filename="meshes/link.stl"/></geometry></visual></link></robot>"#,
        )
        .expect("write URDF");

        let body = SomaBody::load(&yaml_path).expect("load without local mesh");
        assert_eq!(body.urdf_asset_paths.len(), 1);
        assert!(body.read_urdf_assets().is_err());

        std::fs::create_dir(directory.path().join("meshes")).expect("create mesh directory");
        std::fs::write(
            directory.path().join("meshes/link.stl"),
            b"solid link\nendsolid\n",
        )
        .expect("write mesh");
        let assets = body.read_urdf_assets().expect("read requested assets");
        assert_eq!(assets[0].path, "meshes/link.stl");
        assert!(!assets[0].data.is_empty());
    }

    /// Local asset references reject traversal and non-POSIX separators.
    #[test]
    fn rejects_non_posix_or_traversing_asset_paths() {
        assert!(local_asset_path("../outside.stl").is_err());
        assert!(local_asset_path("meshes\\link.stl").is_err());
        assert_eq!(
            local_asset_path("https://example.test/link.stl").expect("external URI"),
            None
        );
    }
}
