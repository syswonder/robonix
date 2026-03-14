// SPDX-License-Identifier: MulanPSL-2.0
// Build Command Module
//
// Build command implementation for robonix-cli

use robonix_cli::manifest::{self, Manifest};
use robonix_cli::output;
use anyhow::{Context, Result};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

const RBNX_BUILD_DIR: &str = "rbnx-build";

#[derive(Debug, Clone)]
pub struct LocalBuildLayout {
    pub package_name: String,
    pub package_root: PathBuf,
    pub install_setup: PathBuf,
}

/// If the package has already been built (rbnx-build/ws/install/setup.bash exists), return its layout so start can skip rebuild.
pub fn get_existing_build_layout(package_root: &Path) -> Result<Option<LocalBuildLayout>> {
    let detected = manifest::detect_and_load(package_root)?;
    let package_name = detected.manifest.package.name.clone();
    let install_setup = package_root
        .join(RBNX_BUILD_DIR)
        .join("ws")
        .join("install")
        .join("setup.bash");
    if install_setup.exists() {
        Ok(Some(LocalBuildLayout {
            package_name,
            package_root: package_root.to_path_buf(),
            install_setup,
        }))
    } else {
        Ok(None)
    }
}

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("robonix-cli should live under repo root")
        .to_path_buf()
}

fn copy_dir_all(src: &Path, dst: &Path) -> Result<()> {
    fs::create_dir_all(dst)?;
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let src_path = entry.path();
        let dst_path = dst.join(entry.file_name());
        if entry.file_type()?.is_dir() {
            copy_dir_all(&src_path, &dst_path)?;
        } else {
            fs::copy(&src_path, &dst_path)?;
        }
    }
    Ok(())
}

fn robonix_interfaces_root_from_catalog(catalog_root: &Path) -> Result<PathBuf> {
    catalog_root
        .parent()
        .map(Path::to_path_buf)
        .context("Interface catalog root should be robonix-interfaces/ridl")
}

/// Write file only when content differs, preserving mtime for incremental colcon builds.
fn write_if_changed(path: &Path, content: &str) -> Result<()> {
    let existing = fs::read_to_string(path).ok();
    if existing.as_deref() == Some(content) {
        return Ok(());
    }
    fs::write(path, content)?;
    Ok(())
}

/// Files to skip when copying (e.g. when we auto-generate them).
/// package.xml is NOT skipped: developers may add their own with custom ROS2 deps.
const SKIP_ROS_PYTHON_FILES: &[&str] = &["setup.py", "setup.cfg"];

/// Sync package to workspace with rsync (preserves mtime for incremental colcon builds).
/// Falls back to copy_package_tree when rsync is not available.
#[cfg(unix)]
fn sync_package_tree(
    src: &Path,
    dst: &Path,
    skip_msgs_package: Option<&str>,
    skip_ros_python_files: bool,
) -> Result<()> {
    fs::create_dir_all(dst)?;
    let mut exclude_args: Vec<String> = vec!["--exclude=rbnx-build".into()];
    if skip_ros_python_files {
        exclude_args.push("--exclude=setup.py".into());
        exclude_args.push("--exclude=setup.cfg".into());
        exclude_args.push("--exclude=resource".into());
    }
    if let Some(name) = skip_msgs_package {
        exclude_args.push(format!("--exclude={name}"));
    }
    let src_trailing = format!("{}/", src.display());
    match Command::new("rsync")
        .args(["-a", "--delete"])
        .args(&exclude_args)
        .arg(&src_trailing)
        .arg(dst)
        .status()
    {
        Ok(status) if status.success() => Ok(()),
        _ => copy_package_tree(src, dst, skip_msgs_package, skip_ros_python_files),
    }
}

#[cfg(not(unix))]
fn sync_package_tree(
    src: &Path,
    dst: &Path,
    skip_msgs_package: Option<&str>,
    skip_ros_python_files: bool,
) -> Result<()> {
    copy_package_tree(src, dst, skip_msgs_package, skip_ros_python_files)
}

fn copy_package_tree(
    src: &Path,
    dst: &Path,
    skip_msgs_package: Option<&str>,
    skip_ros_python_files: bool,
) -> Result<()> {
    fs::create_dir_all(dst)?;
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let file_name = entry.file_name();
        let src_path = entry.path();
        let dst_path = dst.join(&file_name);

        if file_name == RBNX_BUILD_DIR {
            continue;
        }
        if skip_ros_python_files && file_name.to_str() == Some("resource") {
            continue; // resource/ is auto-generated
        }
        if let Some(name) = skip_msgs_package {
            if file_name.to_str() == Some(name) {
                continue;
            }
        }
        if skip_ros_python_files {
            if file_name
                .to_str()
                .map(|s| SKIP_ROS_PYTHON_FILES.contains(&s))
                .unwrap_or(false)
            {
                continue;
            }
        }

        let file_type = entry.file_type()?;
        if file_type.is_dir() {
            copy_package_tree(&src_path, &dst_path, skip_msgs_package, skip_ros_python_files)?;
        } else if file_type.is_file() {
            fs::copy(&src_path, &dst_path).with_context(|| {
                format!(
                    "Failed to copy file from {} to {}",
                    src_path.display(),
                    dst_path.display()
                )
            })?;
        }
    }
    Ok(())
}

#[cfg(unix)]
fn link_current_interfaces(interfaces_root: &Path, build_root: &Path) -> Result<()> {
    use std::os::unix::fs as unix_fs;

    let link_path = build_root.join("interfaces");
    if link_path.exists() {
        if link_path.is_dir() {
            fs::remove_dir_all(&link_path)?;
        } else {
            fs::remove_file(&link_path)?;
        }
    }
    unix_fs::symlink(interfaces_root, &link_path).with_context(|| {
        format!(
            "Failed to create symlink {} -> {}",
            link_path.display(),
            interfaces_root.display()
        )
    })?;
    Ok(())
}

#[cfg(not(unix))]
fn link_current_interfaces(interfaces_root: &Path, build_root: &Path) -> Result<()> {
    let dst = build_root.join("interfaces");
    if dst.exists() {
        fs::remove_dir_all(&dst)?;
    }
    copy_package_tree(interfaces_root, &dst)
}

fn run_shell(command: &str, current_dir: &Path) -> Result<()> {
    #[cfg(unix)]
    let status = Command::new("bash")
        .arg("-lc")
        .arg(command)
        .current_dir(current_dir)
        .status()
        .with_context(|| {
            format!(
                "Failed to execute shell command in {}",
                current_dir.display()
            )
        })?;

    #[cfg(not(unix))]
    let status = Command::new("sh")
        .arg("-lc")
        .arg(command)
        .current_dir(current_dir)
        .status()
        .with_context(|| {
            format!(
                "Failed to execute shell command in {}",
                current_dir.display()
            )
        })?;

    if !status.success() {
        anyhow::bail!(
            "Command failed in {} with exit code {:?}: {}",
            current_dir.display(),
            status.code(),
            command
        );
    }
    Ok(())
}

fn shell_quote(path: &Path) -> String {
    format!("\"{}\"", path.display())
}

/// Collect package names from robonix-interfaces/lib (rcl_interfaces, common_interfaces).
/// These are our maintained ROS msg/srv/action packages; rbnx build uses them via vendor, not system.
fn collect_vendor_msg_packages(interfaces_root: &Path) -> Result<Vec<String>> {
    let lib = interfaces_root.join("lib");
    let mut names = std::collections::BTreeSet::new();
    for sub in ["rcl_interfaces", "common_interfaces"] {
        let sub_path = lib.join(sub);
        if !sub_path.exists() {
            continue;
        }
        for entry in fs::read_dir(&sub_path)? {
            let entry = entry?;
            let pkg_dir = entry.path();
            if !pkg_dir.is_dir() {
                continue;
            }
            let pkg_xml = pkg_dir.join("package.xml");
            if !pkg_xml.exists() {
                continue;
            }
            // Skip meta packages, non-msg packages, and packages with external deps we don't have
            let name = pkg_dir.file_name().and_then(|n| n.to_str()).unwrap_or("").to_string();
            if matches!(name.as_str(), "common_interfaces" | "sensor_msgs_py" | "test_msgs") {
                continue;
            }
            let content = fs::read_to_string(&pkg_xml)?;
            if let Some(start) = content.find("<name>") {
                let rest = &content[start + 6..];
                if let Some(end) = rest.find("</name>") {
                    let pkg_name = rest[..end].trim().to_string();
                    if !pkg_name.is_empty() {
                        names.insert(pkg_name);
                    }
                }
            }
        }
    }
    Ok(names.into_iter().collect())
}

fn build_local(package_root: &Path, manifest: &Manifest, clean: bool) -> Result<LocalBuildLayout> {
    let summary = manifest.validate_and_summarize()?;
    let package_name = summary.name.clone();
    let interface_check = manifest::validate_interface_references(&summary, package_root)?;
    let catalog_root = interface_check.catalog_root.with_context(|| {
        format!(
            "Could not locate robonix-interfaces/ridl near {}",
            package_root.display()
        )
    })?;
    let interfaces_root = robonix_interfaces_root_from_catalog(&catalog_root)?;
    let repo_root = repo_root();
    let ridlc_manifest = repo_root.join("ridlc").join("Cargo.toml");

    let build_root = package_root.join(RBNX_BUILD_DIR);
    let workspace_root = build_root.join("ws");
    if clean && build_root.exists() {
        fs::remove_dir_all(&build_root)
            .with_context(|| format!("Failed to clean {}", build_root.display()))?;
    }
    fs::create_dir_all(build_root.join("logs"))?;
    fs::create_dir_all(workspace_root.join("src").join("package"))?;

    let package_msg_dir = package_root.join("msg");
    let skip_msgs = if package_msg_dir.exists() {
        Some(format!("{package_name}_msgs"))
    } else {
        None
    };
    let has_python_nodes = manifest
        .nodes
        .iter()
        .any(|n| n.entry.as_ref().map_or(false, |e| e.contains(':')));
    sync_package_tree(
        package_root,
        &workspace_root
            .join("src")
            .join("package")
            .join(&package_name),
        skip_msgs.as_deref(),
        has_python_nodes,
    )?;

    // Auto-generate package.xml, setup.py, setup.cfg for Python packages (from robonix_manifest.yaml)
    let package_pkg_root = workspace_root
        .join("src")
        .join("package")
        .join(&package_name);
    if has_python_nodes {
        output::sub_step("Generating ROS2 ament_python package files from manifest");
        let entry_points: Vec<String> = manifest
            .nodes
            .iter()
            .filter_map(|n| {
                let entry = n.entry.as_ref()?;
                let (module, _func) = entry.split_once(':')?;
                let exe_name = module.split('.').last()?;
                Some(format!(r#""{exe_name} = {entry}""#))
            })
            .collect();
        let entry_points_str = if entry_points.is_empty() {
            "{}".to_string()
        } else {
            format!(
                "{{\n        \"console_scripts\": [\n            {},\n        ],\n    }}",
                entry_points.join(",\n            ")
            )
        };
        let package_ridl_dir = package_root.join("ridl");
        let mut deps: Vec<String> = vec![
            "rclpy".into(),
            "robonix_interfaces".into(),
            "robonix_interfaces_ros2".into(),
            "robonix_msgs".into(),
        ];
        // Use our maintained ROS msg packages from robonix-interfaces/lib (ridlc copies to vendor)
        let vendor_pkgs = collect_vendor_msg_packages(&interfaces_root)?;
        deps.extend(vendor_pkgs);
        if package_ridl_dir.exists() {
            deps.push(format!("{package_name}_msgs"));
            deps.push(format!("{package_name}_interfaces"));
            deps.push(format!("{package_name}_interfaces_ros2"));
        }
        let dep_lines: String = deps
            .iter()
            .map(|d| format!("  <depend>{d}</depend>"))
            .collect::<Vec<_>>()
            .join("\n");
        // Only generate package.xml when source has none; otherwise use developer's (for custom ROS2 deps).
        let src_package_xml = package_root.join("package.xml");
        if !src_package_xml.exists() {
            let package_xml = format!(
                r#"<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{package_name}</name>
  <version>{version}</version>
  <description>{description}</description>
  <maintainer email="robonix@example.com">robonix</maintainer>
  <license>{license}</license>
  <buildtool_depend>ament_python</buildtool_depend>
{dep_lines}
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"#,
                version = manifest.package.version,
                description = manifest.package.description,
                license = manifest.package.license,
            );
            write_if_changed(&package_pkg_root.join("package.xml"), &package_xml)?;
        }
        let setup_cfg = format!(
            r#"[develop]
script_dir=$base/lib/{package_name}
[install]
install_scripts=$base/lib/{package_name}
"#
        );
        let setup_py = format!(
            r#"from setuptools import find_packages, setup

package_name = "{package_name}"

setup(
    name=package_name,
    version="{version}",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "grpcio"],
    zip_safe=True,
    maintainer="robonix",
    maintainer_email="robonix@example.com",
    description="{description}",
    license="{license}",
    entry_points={entry_points_str},
)
"#,
            version = manifest.package.version,
            description = manifest.package.description.replace('"', "\\\""),
            license = manifest.package.license,
        );
        write_if_changed(&package_pkg_root.join("setup.cfg"), &setup_cfg)?;
        write_if_changed(&package_pkg_root.join("setup.py"), &setup_py)?;
        let resource_dir = package_pkg_root.join("resource");
        fs::create_dir_all(&resource_dir)?;
        write_if_changed(&resource_dir.join(&package_name), "")?;
    }

    // Auto-generate {package}_msgs from package msg/ directory (avoids manual package.xml, CMakeLists.txt)
    let package_pkg_root = workspace_root
        .join("src")
        .join("package")
        .join(&package_name);
    if package_msg_dir.exists() {
        if let Ok(entries) = fs::read_dir(&package_msg_dir) {
            let msg_files: Vec<_> = entries
                .filter_map(|e| e.ok())
                .filter(|e| {
                    e.path()
                        .extension()
                        .map(|ext| ext == "msg")
                        .unwrap_or(false)
                })
                .collect();
            if !msg_files.is_empty() {
                output::sub_step("Generating package-local msgs from msg/");
                let msgs_pkg_name = format!("{package_name}_msgs");
                let msgs_pkg_dir = package_pkg_root.join(&msgs_pkg_name);
                let msgs_msg_dir = msgs_pkg_dir.join("msg");
                fs::create_dir_all(&msgs_msg_dir)?;
                for entry in &msg_files {
                    let src = entry.path();
                    let name = entry.file_name();
                    fs::copy(&src, msgs_msg_dir.join(&name))?;
                }
                let package_xml = format!(
                    r#"<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{msgs_pkg_name}</name>
  <version>0.1.0</version>
  <description>Auto-generated from package msg/</description>
  <maintainer email="robonix@example.com">robonix</maintainer>
  <license>MulanPSL-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>builtin_interfaces</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"#
                );
                let msg_entries: String = msg_files
                    .iter()
                    .map(|e| {
                        e.file_name()
                            .to_str()
                            .map(|n| format!(r#"  "msg/{n}""#))
                            .unwrap_or_default()
                    })
                    .collect::<Vec<_>>()
                    .join("\n");
                let cmake = format!(
                    r#"cmake_minimum_required(VERSION 3.8)
project({msgs_pkg_name})

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${{PROJECT_NAME}}
{msg_entries}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
"#
                );
                fs::write(msgs_pkg_dir.join("package.xml"), package_xml)?;
                fs::write(msgs_pkg_dir.join("CMakeLists.txt"), cmake)?;
            }
        }
    }

    link_current_interfaces(&interfaces_root, &build_root)?;

    output::sub_step(&format!("Package root: {}", package_root.display()));
    output::sub_step(&format!("Build root: {}", build_root.display()));
    output::sub_step(&format!("Workspace: {}", workspace_root.display()));
    output::sub_step(&format!("Interface catalog: {}", catalog_root.display()));

    let runtime_interfaces = interfaces_root
        .join("lib")
        .join("robonix_runtime_interfaces");
    let rcl_interfaces = interfaces_root.join("lib").join("rcl_interfaces");
    let common_interfaces = interfaces_root.join("lib").join("common_interfaces");

    output::step("Generating", "RIDL workspace into rbnx-build/ws");
    let ridlc_command = format!(
        "cargo run --manifest-path {} -- --lang python --layout workspace -I {} -I {} -I {} -o {} -i {}",
        shell_quote(&ridlc_manifest),
        shell_quote(&runtime_interfaces),
        shell_quote(&rcl_interfaces),
        shell_quote(&common_interfaces),
        shell_quote(&workspace_root),
        shell_quote(&catalog_root),
    );
    run_shell(&ridlc_command, &repo_root)?;

    // Second ridlc run for package-local RIDL (if package has ridl/ directory)
    let package_ridl_dir = workspace_root
        .join("src")
        .join("package")
        .join(&package_name)
        .join("ridl");
    let package_pkg_root = workspace_root
        .join("src")
        .join("package")
        .join(&package_name);
    if package_ridl_dir.exists() {
        output::sub_step("Generating package-local RIDL interfaces");
        let package_output = package_pkg_root.clone();
        let interfaces_pkg_name = format!("{}_interfaces", package_name);
        let ridlc_package_command = format!(
            "cargo run --manifest-path {} -- --lang python -I {} -I {} -I {} -I {} --package-output {} --package-name {} -i {}",
            shell_quote(&ridlc_manifest),
            shell_quote(&runtime_interfaces),
            shell_quote(&rcl_interfaces),
            shell_quote(&common_interfaces),
            shell_quote(&package_pkg_root),
            shell_quote(&package_output),
            interfaces_pkg_name,
            shell_quote(&package_ridl_dir),
        );
        run_shell(&ridlc_package_command, &repo_root)?;
        // Merge {pkg}_interfaces/skill_demo/skill/ into main package so skill_demo.skill is importable
        let interfaces_skill = package_pkg_root
            .join(format!("{package_name}_interfaces"))
            .join(&package_name)
            .join("skill");
        let main_skill = package_pkg_root.join(&package_name).join("skill");
        if interfaces_skill.is_dir() {
            fs::create_dir_all(main_skill.parent().unwrap())?;
            if main_skill.exists() {
                fs::remove_dir_all(&main_skill)?;
            }
            copy_dir_all(&interfaces_skill, &main_skill)?;
        }
        // Exclude {pkg} from {pkg}_interfaces so main package wins in PYTHONPATH
        let interfaces_setup = package_pkg_root
            .join(format!("{package_name}_interfaces"))
            .join("setup.py");
        if let Ok(content) = fs::read_to_string(&interfaces_setup) {
            let patched = content.replace(
                "packages=find_packages(exclude=['test'])",
                &format!("packages=find_packages(exclude=['test', '{package_name}'])"),
            );
            if patched != content {
                fs::write(&interfaces_setup, patched)?;
            }
        }
    }

    output::step("Building", "colcon workspace under rbnx-build/ws");
    let distro = std::env::var("ROS_DISTRO").unwrap_or_else(|_| "humble".to_string());
    let ros_setup = format!("/opt/ros/{distro}/setup.bash");
    // When package has ridl/, add nested package dirs to base-paths so colcon finds {pkg}_msgs, {pkg}_interfaces, etc.
    let pkg_base = if package_ridl_dir.exists() {
        let mut extra = String::new();
        for sub in [
            format!("{package_name}_msgs"),
            format!("{package_name}_interfaces"),
            format!("{package_name}_interfaces_ros2"),
        ] {
            let p = package_pkg_root.join(&sub);
            if p.exists() {
                extra.push(' ');
                extra.push_str(&shell_quote(&p));
            }
        }
        extra
    } else {
        String::new()
    };
    // Avoid setuptools/packaging canonicalize_version mismatch; do not set PYTHONNOUSERSITE so ~/.local is visible
    let colcon_command = format!(
        "pip3 install --user --upgrade 'packaging>=22.0' 'setuptools>=72' 2>/dev/null || true; set +u; source {ros_setup}; set -u; colcon --log-base {log_base} build --base-paths {generated} {vendor} {app} {package}{pkg_base} --build-base {build_base} --install-base {install_base} --packages-up-to robonix_interfaces_app {pkg}",
        ros_setup = shell_quote(Path::new(&ros_setup)),
        generated = shell_quote(&workspace_root.join("src").join("generated")),
        vendor = shell_quote(&workspace_root.join("src").join("vendor")),
        app = shell_quote(&workspace_root.join("src").join("app")),
        package = shell_quote(&workspace_root.join("src").join("package")),
        pkg_base = pkg_base,
        build_base = shell_quote(&workspace_root.join("build")),
        install_base = shell_quote(&workspace_root.join("install")),
        log_base = shell_quote(&workspace_root.join("log")),
        pkg = package_name,
    );
    run_shell(&colcon_command, package_root)?;

    output::check(&format!(
        "Generated workspace: {}",
        workspace_root.display()
    ));
    output::check(&format!(
        "Install setup: {}",
        workspace_root.join("install").join("setup.bash").display()
    ));
    output::success(&format!(
        "Package '{}' built into {}",
        summary.name,
        build_root.display()
    ));
    Ok(LocalBuildLayout {
        package_name: summary.name,
        package_root: package_root.to_path_buf(),
        install_setup: workspace_root.join("install").join("setup.bash"),
    })
}

pub fn build_local_package(path: &Path, clean: bool) -> Result<LocalBuildLayout> {
    let package_root = path
        .canonicalize()
        .with_context(|| format!("Failed to canonicalize package path {}", path.display()))?;

    let detected = manifest::detect_and_load(&package_root)?;
    build_local(&package_root, &detected.manifest, clean)
}

pub async fn execute_local(path: PathBuf, clean: bool) -> Result<()> {
    let package_root = path
        .canonicalize()
        .with_context(|| format!("Failed to canonicalize package path {}", path.display()))?;
    output::action(
        "Building",
        &format!("local package at {}", package_root.display()),
    );

    build_local_package(&package_root, clean)?;
    Ok(())
}

