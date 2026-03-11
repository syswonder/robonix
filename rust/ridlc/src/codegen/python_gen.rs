// SPDX-License-Identifier: MulanPSL-2.0
// Python ROS2 code generator for RIDL (PoC)
// Generates rclpy-based client/server stubs for query, stream, command, event.

use anyhow::{Context, Result, bail};
use std::collections::BTreeMap;
use std::fs;
use std::io::{self, IsTerminal};
use std::path::{Path, PathBuf};

use crate::ast::{CommandDef, EventDef, File, Interface, QueryDef, StreamDef, StreamDirection};

fn ridlc_prefix() -> &'static str {
    if io::stderr().is_terminal() {
        "\x1b[1;38;5;45m[ridlc]\x1b[0m"
    } else {
        "[ridlc]"
    }
}

/// Path to proto/gen (gRPC Python stubs) relative to ridlc crate root.
fn proto_gen_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("proto").join("gen")
}

/// Copy gRPC runtime client modules into out_dir so generated code works out of the box.
/// Call once per --lang python run. If proto/gen is missing, log and skip (user must run gen_grpc.sh).
pub fn emit_runtime_grpc(out_dir: &Path) -> Result<()> {
    fs::create_dir_all(out_dir)?;
    let gen_dir = proto_gen_dir();
    let files = ["robonix_runtime_pb2.py", "robonix_runtime_pb2_grpc.py"];
    for f in &files {
        let src = gen_dir.join(f);
        if !src.exists() {
            eprintln!(
                "{} warning: {} not found (run proto/gen_grpc.sh in ridlc repo); runtime helpers will fail at import",
                ridlc_prefix(),
                src.display()
            );
            return Ok(());
        }
        let dst = out_dir.join(f);
        fs::copy(&src, &dst).with_context(|| format!("copy {} -> {}", src.display(), dst.display()))?;
    }
    eprintln!("{} emitted gRPC runtime client into {}", ridlc_prefix(), out_dir.display());
    Ok(())
}

/// Emit ROS2 ament_python package files (package.xml, setup.cfg, setup.py) so out_dir
/// is a valid ROS2 package and can be built with colcon.
pub fn emit_ros_package_files(out_dir: &Path, package_name: &str) -> Result<()> {
    let package_xml = r#"<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>PACKAGE_NAME</name>
  <version>0.0.1</version>
  <description>RIDL-generated interfaces and runtime gRPC client (robonix meta API).</description>
  <maintainer email="wheatfox17@icloud.com">robonix</maintainer>
  <license>MulanPSL-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"#.replace("PACKAGE_NAME", package_name);

    let setup_cfg = r#"[develop]
script_dir=$base/lib/PACKAGE_NAME
[install]
install_scripts=$base/lib/PACKAGE_NAME
"#.replace("PACKAGE_NAME", package_name);

    let setup_py = r#"from setuptools import setup, find_packages

package_name = 'PACKAGE_NAME'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    py_modules=['robonix_runtime_pb2', 'robonix_runtime_pb2_grpc'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'grpcio'],
    zip_safe=True,
    maintainer='robonix',
    maintainer_email='wheatfox17@icloud.com',
    description='RIDL-generated interfaces and runtime gRPC client',
    license='MulanPSL-2.0',
    tests_require=['pytest'],
    entry_points={},
)
"#.replace("PACKAGE_NAME", package_name);

    fs::write(out_dir.join("package.xml"), package_xml)?;
    fs::write(out_dir.join("setup.cfg"), setup_cfg)?;
    fs::write(out_dir.join("setup.py"), setup_py)?;
    let resource_dir = out_dir.join("resource");
    fs::create_dir_all(&resource_dir)?;
    let marker = resource_dir.join(package_name);
    fs::write(marker, "")?;
    eprintln!(
        "{} emitted ROS2 package files (package.xml, setup.cfg, setup.py) for '{}'",
        ridlc_prefix(),
        package_name
    );
    Ok(())
}

/// Map RIDL type ref (e.g. "geometry_msgs/msg/Twist", "robonix_msg/msg/Object[]")
/// to ROS2 Python import and type name. Strips [] from type name; maps robonix_msg -> robonix_msgs.
fn ros2_python_type(type_ref: &str) -> (String, String) {
    let trimmed = type_ref.trim();
    let parts: Vec<&str> = trimmed.split('/').collect();
    if parts.len() >= 3 {
        let mut pkg = parts[0];
        if pkg == "robonix_msg" {
            pkg = "robonix_msgs";
        }
        let mut type_name = parts[2].to_string();
        if type_name.ends_with("[]") {
            type_name.truncate(type_name.len().saturating_sub(2));
        }
        (format!("{}.msg", pkg), type_name)
    } else {
        ("std_msgs.msg".to_string(), trimmed.to_string())
    }
}

const PYTHON_PACKAGE_NAME: &str = "robonix_interfaces";
const ROSIDL_PACKAGE_NAME: &str = "robonix_interfaces_ros2";
const ROBONIX_MSGS_PACKAGE_NAME: &str = "robonix_msgs";
const GENERATED_DIR_NAME: &str = "generated";
const VENDOR_DIR_NAME: &str = "vendor";
const APP_DIR_NAME: &str = "app";
const APP_PACKAGE_NAME: &str = "robonix_interfaces_app";

/// Convert a RIDL namespace like `robonix/prm/base` into a stable type prefix
/// like `PrmBase`. The root `robonix` segment is omitted to keep symbols shorter.
fn namespace_type_prefix(ns: &str) -> String {
    ns.split('/')
        .filter(|part| !part.is_empty() && *part != "robonix")
        .map(pascal)
        .collect::<String>()
}

/// Build a ROS IDL symbol name from namespace + interface name.
/// Example: `robonix/prm/base` + `get_status` -> `PrmBaseGetStatus`
fn rosidl_symbol_name(ns: &str, iface_name: &str) -> String {
    format!("{}{}", namespace_type_prefix(ns), pascal(iface_name))
}

fn rosidl_root(out_dir: &Path) -> PathBuf {
    out_dir.join("rosidl")
}

fn sorted_files(dir: &Path) -> Result<Vec<PathBuf>> {
    let mut files = Vec::new();
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let path = entry.path();
        if path.is_file() {
            files.push(path);
        }
    }
    files.sort();
    Ok(files)
}

fn copy_dir_recursive(src: &Path, dst: &Path) -> Result<()> {
    if dst.exists() {
        fs::remove_dir_all(dst)?;
    }
    fs::create_dir_all(dst)?;
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let src_path = entry.path();
        let dst_path = dst.join(entry.file_name());
        if src_path.is_dir() {
            copy_dir_recursive(&src_path, &dst_path)?;
        } else {
            fs::copy(&src_path, &dst_path)
                .with_context(|| format!("copy {} -> {}", src_path.display(), dst_path.display()))?;
        }
    }
    Ok(())
}

fn package_name_from_xml(package_xml: &Path) -> Result<String> {
    let content = fs::read_to_string(package_xml)
        .with_context(|| format!("read package xml '{}'", package_xml.display()))?;
    let start = content
        .find("<name>")
        .with_context(|| format!("missing <name> in '{}'", package_xml.display()))?;
    let end = content[start + 6..]
        .find("</name>")
        .with_context(|| format!("missing </name> in '{}'", package_xml.display()))?;
    Ok(content[start + 6..start + 6 + end].trim().to_string())
}

fn maybe_copy_ros_package(src_dir: &Path, workspace_src: &Path) -> Result<Option<String>> {
    let package_xml = src_dir.join("package.xml");
    if !package_xml.exists() {
        return Ok(None);
    }

    let package_name = package_name_from_xml(&package_xml)?;
    if matches!(
        package_name.as_str(),
        PYTHON_PACKAGE_NAME | ROSIDL_PACKAGE_NAME | ROBONIX_MSGS_PACKAGE_NAME
    ) {
        return Ok(None);
    }

    let dst = workspace_src.join(&package_name);
    copy_dir_recursive(src_dir, &dst)?;
    Ok(Some(package_name))
}

fn vendor_include_packages(workspace_src: &Path, include_paths: &[PathBuf]) -> Result<Vec<String>> {
    let vendor_root = workspace_src.join(VENDOR_DIR_NAME);
    fs::create_dir_all(&vendor_root)?;
    let mut copied = std::collections::BTreeSet::new();
    for include_path in include_paths {
        if !include_path.exists() {
            continue;
        }

        if let Some(package_name) = maybe_copy_ros_package(include_path, &vendor_root)? {
            copied.insert(package_name);
            continue;
        }

        for entry in fs::read_dir(include_path)? {
            let entry = entry?;
            let path = entry.path();
            if !path.is_dir() {
                continue;
            }
            if let Some(package_name) = maybe_copy_ros_package(&path, &vendor_root)? {
                copied.insert(package_name);
            }
        }
    }
    Ok(copied.into_iter().collect())
}

fn type_dependency_from_token(token: &str) -> Option<String> {
    let parts: Vec<&str> = token.split('/').collect();
    match parts.as_slice() {
        [pkg, kind, _name] if matches!(*kind, "msg" | "srv" | "action") => Some((*pkg).to_string()),
        [pkg, _name] => Some((*pkg).to_string()),
        _ => None,
    }
}

fn collect_rosidl_metadata(
    pkg_dir: &Path,
    subdirs: &[&str],
    self_pkg: &str,
) -> Result<(Vec<String>, Vec<String>)> {
    let mut deps = std::collections::BTreeSet::new();
    let mut interface_files = Vec::new();
    let mut has_action = false;

    for subdir in subdirs {
        let src_dir = pkg_dir.join(subdir);
        if !src_dir.exists() {
            continue;
        }
        for path in sorted_files(&src_dir)? {
            let file_name = path
                .file_name()
                .and_then(|name| name.to_str())
                .unwrap_or_default()
                .to_string();
            interface_files.push(format!("{subdir}/{file_name}"));
            if path.extension().and_then(|ext| ext.to_str()) == Some("action") {
                has_action = true;
            }
            let content = fs::read_to_string(&path)?;
            for raw_line in content.lines() {
                let line = raw_line.trim();
                if line.is_empty() || line.starts_with('#') || line == "---" {
                    continue;
                }
                let token = line.split_whitespace().next().unwrap_or_default();
                if let Some(dep) = type_dependency_from_token(token) {
                    if dep != self_pkg {
                        deps.insert(dep);
                    }
                }
            }
        }
    }

    if has_action {
        deps.insert("action_msgs".to_string());
    }

    Ok((deps.into_iter().collect(), interface_files))
}

fn write_rosidl_package_files(
    out_dir: &Path,
    package_name: &str,
    description: &str,
    deps: &[String],
    interface_files: &[String],
) -> Result<()> {
    let depend_xml = deps
        .iter()
        .map(|dep| format!("  <depend>{dep}</depend>"))
        .collect::<Vec<_>>()
        .join("\n");
    let package_xml = format!(
        "<?xml version=\"1.0\"?>\n\
<?xml-model href=\"http://download.ros.org/schema/package_format3.xsd\" schematypens=\"http://www.w3.org/2001/XMLSchema\"?>\n\
<package format=\"3\">\n\
  <name>{package_name}</name>\n\
  <version>0.0.1</version>\n\
  <description>{description}</description>\n\
  <maintainer email=\"wheatfox17@icloud.com\">robonix</maintainer>\n\
  <license>MulanPSL-2.0</license>\n\
  <buildtool_depend>ament_cmake</buildtool_depend>\n\
  <buildtool_depend>rosidl_default_generators</buildtool_depend>\n\
{depend_xml}\n\
  <exec_depend>rosidl_default_runtime</exec_depend>\n\
  <member_of_group>rosidl_interface_packages</member_of_group>\n\
</package>\n"
    );

    let find_packages = deps
        .iter()
        .map(|dep| format!("find_package({dep} REQUIRED)"))
        .collect::<Vec<_>>()
        .join("\n");
    let interfaces_block = interface_files
        .iter()
        .map(|entry| format!("  \"{entry}\""))
        .collect::<Vec<_>>()
        .join("\n");
    let dependency_clause = if deps.is_empty() {
        String::new()
    } else {
        format!("  DEPENDENCIES {}\n", deps.join(" "))
    };
    let cmake = format!(
        "cmake_minimum_required(VERSION 3.8)\n\
project({package_name})\n\n\
find_package(ament_cmake REQUIRED)\n\
find_package(rosidl_default_generators REQUIRED)\n\
{find_packages}\n\n\
rosidl_generate_interfaces(${{PROJECT_NAME}}\n\
{interfaces_block}\n\
{dependency_clause})\n\n\
ament_export_dependencies(rosidl_default_runtime)\n\
ament_package()\n"
    );

    fs::write(out_dir.join("package.xml"), package_xml)?;
    fs::write(out_dir.join("CMakeLists.txt"), cmake)?;
    Ok(())
}

pub fn assemble_workspace_ros_packages(
    workspace_src: &Path,
    python_pkg_dir: &Path,
    include_paths: &[PathBuf],
) -> Result<()> {
    let rosidl_out = rosidl_root(python_pkg_dir);
    let generated_root = workspace_src.join(GENERATED_DIR_NAME);
    let vendor_root = workspace_src.join(VENDOR_DIR_NAME);
    fs::create_dir_all(&generated_root)?;
    fs::create_dir_all(&vendor_root)?;
    let _ = vendor_include_packages(workspace_src, include_paths)?;

    let robonix_msgs_src = rosidl_out.join(ROBONIX_MSGS_PACKAGE_NAME);
    if robonix_msgs_src.exists() {
        let robonix_msgs_dst = vendor_root.join(ROBONIX_MSGS_PACKAGE_NAME);
        let msg_src = robonix_msgs_src.join("msg");
        if msg_src.exists() {
            copy_dir_recursive(&msg_src, &robonix_msgs_dst.join("msg"))?;
        }
        let (deps, interface_files) =
            collect_rosidl_metadata(&robonix_msgs_src, &["msg"], ROBONIX_MSGS_PACKAGE_NAME)?;
        write_rosidl_package_files(
            &robonix_msgs_dst,
            ROBONIX_MSGS_PACKAGE_NAME,
            "Robonix message definitions generated from checked-in IDL.",
            &deps,
            &interface_files,
        )?;
    }

    let ros2_src = rosidl_out.join(ROSIDL_PACKAGE_NAME);
    if ros2_src.exists() {
        let ros2_dst = generated_root.join(ROSIDL_PACKAGE_NAME);
        for subdir in ["msg", "srv", "action"] {
            let src_dir = ros2_src.join(subdir);
            if src_dir.exists() {
                copy_dir_recursive(&src_dir, &ros2_dst.join(subdir))?;
            }
        }
        let (deps, interface_files) =
            collect_rosidl_metadata(&ros2_src, &["msg", "srv", "action"], ROSIDL_PACKAGE_NAME)?;
        write_rosidl_package_files(
            &ros2_dst,
            ROSIDL_PACKAGE_NAME,
            "RIDL-generated ROS interface package.",
            &deps,
            &interface_files,
        )?;
    }

    if rosidl_out.exists() {
        fs::remove_dir_all(&rosidl_out)?;
    }

    Ok(())
}

fn emit_app_package_files(out_dir: &Path, entry_points: &[(String, String)]) -> Result<()> {
    let package_xml = r#"<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>PACKAGE_NAME</name>
  <version>0.0.1</version>
  <description>User-editable app skeletons for RIDL-generated interfaces.</description>
  <maintainer email="wheatfox17@icloud.com">robonix</maintainer>
  <license>MulanPSL-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>robonix_interfaces</depend>
  <depend>robonix_interfaces_ros2</depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"#
    .replace("PACKAGE_NAME", APP_PACKAGE_NAME);

    let setup_cfg = r#"[develop]
script_dir=$base/lib/PACKAGE_NAME
[install]
install_scripts=$base/lib/PACKAGE_NAME
"#
    .replace("PACKAGE_NAME", APP_PACKAGE_NAME);

    let entry_points_block = if entry_points.is_empty() {
        String::new()
    } else {
        let lines = entry_points
            .iter()
            .map(|(name, target)| format!("            '{} = {}',", name, target))
            .collect::<Vec<_>>()
            .join("\n");
        format!(
            "    entry_points={{\n        'console_scripts': [\n{}\n        ],\n    }},\n",
            lines
        )
    };

    let setup_py = format!(
        "from setuptools import find_packages, setup\n\npackage_name = '{}'\n\nsetup(\n    name=package_name,\n    version='0.0.1',\n    packages=find_packages(exclude=['test']),\n    data_files=[\n        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),\n        ('share/' + package_name, ['package.xml', 'README.md']),\n    ],\n    install_requires=['setuptools', 'grpcio'],\n    zip_safe=True,\n    maintainer='robonix',\n    maintainer_email='wheatfox17@icloud.com',\n    description='User-editable app skeletons for RIDL-generated interfaces',\n    license='MulanPSL-2.0',\n    tests_require=['pytest'],\n{}{}\n",
        APP_PACKAGE_NAME,
        entry_points_block,
        ")"
    );

    let readme = r#"# robonix_interfaces_app

This package is the user-editable layer for a RIDL-generated workspace.

- Edit code under `src/app/robonix_interfaces_app`
- Do not edit `src/generated` or `src/vendor`
- Each generated module is a starting point for a long-running node that binds to the runtime meta API and ROS transport
"#;

    fs::write(out_dir.join("package.xml"), package_xml)?;
    fs::write(out_dir.join("setup.cfg"), setup_cfg)?;
    fs::write(out_dir.join("setup.py"), setup_py)?;
    fs::write(out_dir.join("README.md"), readme)?;
    let resource_dir = out_dir.join("resource");
    fs::create_dir_all(&resource_dir)?;
    fs::write(resource_dir.join(APP_PACKAGE_NAME), "")?;
    Ok(())
}

fn ensure_python_package_tree(root_dir: &Path, namespace: &str) -> Result<PathBuf> {
    let mut pkg_dir = root_dir.to_path_buf();
    for part in namespace.split('/').filter(|part| !part.is_empty()) {
        pkg_dir = pkg_dir.join(part);
        fs::create_dir_all(&pkg_dir)?;
        let init_path = pkg_dir.join("__init__.py");
        if !init_path.exists() {
            fs::write(&init_path, "# Generated app package.\n")?;
        }
    }
    Ok(pkg_dir)
}

fn app_entry_name(namespace: &str, iface_name: &str, role: &str) -> String {
    let prefix = namespace
        .split('/')
        .filter(|part| !part.is_empty() && *part != "robonix")
        .collect::<Vec<_>>()
        .join("_");
    if prefix.is_empty() {
        format!("{}_{}", iface_name, role)
    } else {
        format!("{}_{}_{}", prefix, iface_name, role)
    }
}

fn emit_query_app_module(out_dir: &Path, namespace: &str, q: &QueryDef) -> Result<String> {
    let ns_import = namespace.replace('/', ".");
    let module_name = format!("{}_server", q.name);
    let mut out = String::new();
    out.push_str("# Generated by ridlc. Edit this file to add your app logic.\n\n");
    out.push_str("import os\n\n");
    out.push_str("import grpc\n");
    out.push_str("import rclpy\n\n");
    out.push_str(&format!(
        "from {} import create_{}_server\n",
        ns_import, q.name
    ));
    out.push_str("from robonix_runtime_pb2_grpc import RobonixRuntimeStub\n\n\n");
    out.push_str("def main() -> None:\n");
    out.push_str("    endpoint = os.environ.get(\"ROBONIX_RUNTIME_ENDPOINT\", \"127.0.0.1:50051\")\n");
    out.push_str(&format!(
        "    node_id = os.environ.get(\"ROBONIX_NODE_ID\", \"{}\")\n\n",
        app_entry_name(namespace, &q.name, "server")
    ));
    out.push_str("    grpc_channel = grpc.insecure_channel(endpoint)\n");
    out.push_str("    runtime_client = RobonixRuntimeStub(grpc_channel)\n");
    out.push_str(&format!(
        "    server = create_{}_server(runtime_client, node_id=node_id)\n\n",
        q.name
    ));
    out.push_str("    def handler(request, response):\n");
    out.push_str("        # TODO: replace this stub logic with your real implementation.\n");
    out.push_str("        return response\n\n");
    out.push_str("    server.start(handler)\n\n");
    out.push_str("    try:\n");
    out.push_str("        rclpy.spin(server)\n");
    out.push_str("    finally:\n");
    out.push_str("        server.destroy_node()\n");
    out.push_str("        if rclpy.ok():\n");
    out.push_str("            rclpy.shutdown()\n\n\n");
    out.push_str("if __name__ == \"__main__\":\n");
    out.push_str("    main()\n");
    fs::write(out_dir.join(format!("{}.py", module_name)), out)?;
    Ok(module_name)
}

fn emit_command_app_module(out_dir: &Path, namespace: &str, c: &CommandDef) -> Result<String> {
    let ns_import = namespace.replace('/', ".");
    let module_name = format!("{}_server", c.name);
    let mut out = String::new();
    out.push_str("# Generated by ridlc. Edit this file to add your app logic.\n\n");
    out.push_str("import os\n\n");
    out.push_str("import grpc\n");
    out.push_str("import rclpy\n\n");
    out.push_str(&format!(
        "from {} import create_{}_server\n",
        ns_import, c.name
    ));
    out.push_str("from robonix_runtime_pb2_grpc import RobonixRuntimeStub\n\n\n");
    out.push_str("def main() -> None:\n");
    out.push_str("    endpoint = os.environ.get(\"ROBONIX_RUNTIME_ENDPOINT\", \"127.0.0.1:50051\")\n");
    out.push_str(&format!(
        "    node_id = os.environ.get(\"ROBONIX_NODE_ID\", \"{}\")\n\n",
        app_entry_name(namespace, &c.name, "server")
    ));
    out.push_str("    grpc_channel = grpc.insecure_channel(endpoint)\n");
    out.push_str("    runtime_client = RobonixRuntimeStub(grpc_channel)\n");
    out.push_str(&format!(
        "    server = create_{}_server(runtime_client, node_id=node_id)\n\n",
        c.name
    ));
    out.push_str("    def execute(request):\n");
    out.push_str("        # TODO: inspect request and populate the action result.\n");
    out.push_str("        result = server._action_type.Result()\n");
    out.push_str("        return result\n\n");
    out.push_str("    server.execute = execute\n");
    out.push_str("    server.start()\n\n");
    out.push_str("    try:\n");
    out.push_str("        rclpy.spin(server)\n");
    out.push_str("    finally:\n");
    out.push_str("        server.destroy_node()\n");
    out.push_str("        if rclpy.ok():\n");
    out.push_str("            rclpy.shutdown()\n\n\n");
    out.push_str("if __name__ == \"__main__\":\n");
    out.push_str("    main()\n");
    fs::write(out_dir.join(format!("{}.py", module_name)), out)?;
    Ok(module_name)
}

fn emit_stream_app_module(out_dir: &Path, namespace: &str, s: &StreamDef) -> Result<(String, &'static str)> {
    let ns_import = namespace.replace('/', ".");
    let field = s
        .fields
        .first()
        .ok_or_else(|| anyhow::anyhow!("stream '{}' must have one field", s.name))?;
    let (module_name, role, body) = match field.direction {
        StreamDirection::Output => {
            let module_name = format!("{}_publisher", s.name);
            let mut body = String::new();
            body.push_str("    publisher = create_");
            body.push_str(&s.name);
            body.push_str("_publisher(runtime_client, node_id=node_id)\n\n");
            body.push_str("    def publish_once():\n");
            body.push_str("        # TODO: populate the message fields before publishing.\n");
            body.push_str("        msg = publisher._msg_type()\n");
            body.push_str("        publisher.publish(msg)\n\n");
            body.push_str("    timer = publisher.create_timer(0.5, publish_once)\n\n");
            body.push_str("    try:\n");
            body.push_str("        rclpy.spin(publisher)\n");
            body.push_str("    finally:\n");
            body.push_str("        timer.cancel()\n");
            body.push_str("        publisher.destroy_node()\n");
            body.push_str("        if rclpy.ok():\n");
            body.push_str("            rclpy.shutdown()\n");
            (module_name, "publisher", body)
        }
        StreamDirection::Input => {
            let module_name = format!("{}_subscriber", s.name);
            let mut body = String::new();
            body.push_str("    target = os.environ.get(\"ROBONIX_TARGET\", \"*\")\n");
            body.push_str("    subscriber = create_");
            body.push_str(&s.name);
            body.push_str("_subscriber(runtime_client, requester_id=node_id, target=target)\n\n");
            body.push_str("    def on_message(msg):\n");
            body.push_str("        # TODO: handle inbound stream data here.\n");
            body.push_str("        subscriber.get_logger().info(f\"received message: {msg}\")\n\n");
            body.push_str("    subscriber.start(on_message)\n\n");
            body.push_str("    try:\n");
            body.push_str("        rclpy.spin(subscriber)\n");
            body.push_str("    finally:\n");
            body.push_str("        subscriber.destroy_node()\n");
            body.push_str("        if rclpy.ok():\n");
            body.push_str("            rclpy.shutdown()\n");
            (module_name, "subscriber", body)
        }
    };

    let mut out = String::new();
    out.push_str("# Generated by ridlc. Edit this file to add your app logic.\n\n");
    out.push_str("import os\n\n");
    out.push_str("import grpc\n");
    out.push_str("import rclpy\n\n");
    out.push_str(&format!(
        "from {} import create_{}_{}\n",
        ns_import, s.name, role
    ));
    out.push_str("from robonix_runtime_pb2_grpc import RobonixRuntimeStub\n\n\n");
    out.push_str("def main() -> None:\n");
    out.push_str("    endpoint = os.environ.get(\"ROBONIX_RUNTIME_ENDPOINT\", \"127.0.0.1:50051\")\n");
    out.push_str(&format!(
        "    node_id = os.environ.get(\"ROBONIX_NODE_ID\", \"{}\")\n\n",
        app_entry_name(namespace, &s.name, role)
    ));
    out.push_str("    grpc_channel = grpc.insecure_channel(endpoint)\n");
    out.push_str("    runtime_client = RobonixRuntimeStub(grpc_channel)\n");
    out.push_str(&body);
    out.push_str("\n\nif __name__ == \"__main__\":\n");
    out.push_str("    main()\n");
    fs::write(out_dir.join(format!("{}.py", module_name)), out)?;
    Ok((module_name, role))
}

fn emit_event_app_module(out_dir: &Path, namespace: &str, e: &EventDef) -> Result<String> {
    let ns_import = namespace.replace('/', ".");
    let module_name = format!("{}_publisher", e.name);
    let mut out = String::new();
    out.push_str("# Generated by ridlc. Edit this file to add your app logic.\n\n");
    out.push_str("import rclpy\n\n");
    out.push_str(&format!("from {} import {}Publisher\n\n\n", ns_import, pascal(&e.name)));
    out.push_str("def main() -> None:\n");
    out.push_str("    publisher = ");
    out.push_str(&format!("{}Publisher(\"/todo/event\")\n", pascal(&e.name)));
    out.push_str("    msg = publisher._msg_type()\n");
    out.push_str("    publisher.emit(msg)\n");
    out.push_str("    publisher.destroy_node()\n");
    out.push_str("    if rclpy.ok():\n");
    out.push_str("        rclpy.shutdown()\n\n\n");
    out.push_str("if __name__ == \"__main__\":\n");
    out.push_str("    main()\n");
    fs::write(out_dir.join(format!("{}.py", module_name)), out)?;
    Ok(module_name)
}

fn emit_combined_runtime_module(py_root: &Path, files_by_ns: &BTreeMap<String, File>) -> Result<String> {
    let module_name = "combined_runtime";
    let mut import_lines = Vec::new();
    let mut setup_lines = Vec::new();
    let mut loop_lines = Vec::new();
    let mut teardown_lines = Vec::new();

    for (namespace, ast) in files_by_ns {
        let ns_import = namespace.replace('/', ".");
        for iface in &ast.interfaces {
            match iface {
                Interface::Query(q) => {
                    import_lines.push(format!(
                        "from {} import create_{}_server",
                        ns_import, q.name
                    ));
                    let entry = app_entry_name(namespace, &q.name, "server");
                    let env_name = format!("ROBONIX_NODE_ID_{}", entry.to_uppercase());
                    setup_lines.push(format!(
                        "    {name}_server = create_{name}_server(runtime_client, node_id=os.environ.get(\"{env}\", \"{entry}\"))",
                        name = q.name,
                        env = env_name,
                        entry = entry
                    ));
                    setup_lines.push(format!(
                        "    def {name}_handler(request, response):",
                        name = q.name
                    ));
                    setup_lines.push("        # TODO: fill the response with real query logic.".to_string());
                    setup_lines.push("        return response".to_string());
                    setup_lines.push(format!("    {name}_server.start({name}_handler)", name = q.name));
                    setup_lines.push(format!("    nodes.append({}_server)", q.name));
                }
                Interface::Command(c) => {
                    import_lines.push(format!(
                        "from {} import create_{}_server",
                        ns_import, c.name
                    ));
                    let entry = app_entry_name(namespace, &c.name, "server");
                    let env_name = format!("ROBONIX_NODE_ID_{}", entry.to_uppercase());
                    setup_lines.push(format!(
                        "    {name}_server = create_{name}_server(runtime_client, node_id=os.environ.get(\"{env}\", \"{entry}\"))",
                        name = c.name,
                        env = env_name,
                        entry = entry
                    ));
                    setup_lines.push(format!("    def {name}_execute(request):", name = c.name));
                    setup_lines.push("        # TODO: inspect request and populate the action result.".to_string());
                    setup_lines.push(format!("        result = {name}_server._action_type.Result()", name = c.name));
                    setup_lines.push("        return result".to_string());
                    setup_lines.push(format!("    {name}_server.execute = {name}_execute", name = c.name));
                    setup_lines.push(format!("    {name}_server.start()", name = c.name));
                    setup_lines.push(format!("    nodes.append({}_server)", c.name));
                }
                Interface::Stream(s) => {
                    let field = s
                        .fields
                        .first()
                        .ok_or_else(|| anyhow::anyhow!("stream '{}' must have one field", s.name))?;
                    match field.direction {
                        StreamDirection::Output => {
                            import_lines.push(format!(
                                "from {} import create_{}_publisher",
                                ns_import, s.name
                            ));
                            let entry = app_entry_name(namespace, &s.name, "publisher");
                            let env_name = format!("ROBONIX_NODE_ID_{}", entry.to_uppercase());
                            setup_lines.push(format!(
                                "    {name}_publisher = create_{name}_publisher(runtime_client, node_id=os.environ.get(\"{env}\", \"{entry}\"))",
                                name = s.name,
                                env = env_name,
                                entry = entry
                            ));
                            setup_lines.push(format!("    def publish_{name}_once():", name = s.name));
                            setup_lines.push("        # TODO: populate and publish the stream message.".to_string());
                            setup_lines.push(format!("        msg = {name}_publisher._msg_type()", name = s.name));
                            setup_lines.push(format!("        {name}_publisher.publish(msg)", name = s.name));
                            setup_lines.push(format!(
                                "    {name}_timer = {name}_publisher.create_timer(0.5, publish_{name}_once)",
                                name = s.name
                            ));
                            setup_lines.push(format!("    nodes.append({}_publisher)", s.name));
                            setup_lines.push(format!("    timers.append({}_timer)", s.name));
                        }
                        StreamDirection::Input => {
                            import_lines.push(format!(
                                "from {} import create_{}_subscriber",
                                ns_import, s.name
                            ));
                            let entry = app_entry_name(namespace, &s.name, "subscriber");
                            let node_env = format!("ROBONIX_NODE_ID_{}", entry.to_uppercase());
                            let target_env = format!("ROBONIX_TARGET_{}", entry.to_uppercase());
                            setup_lines.push(format!(
                                "    {name}_subscriber = create_{name}_subscriber(runtime_client, requester_id=os.environ.get(\"{node_env}\", \"{entry}\"), target=os.environ.get(\"{target_env}\", \"*\"))",
                                name = s.name,
                                node_env = node_env,
                                target_env = target_env,
                                entry = entry
                            ));
                            setup_lines.push(format!("    def on_{name}_message(msg):", name = s.name));
                            setup_lines.push(format!(
                                "        {name}_subscriber.get_logger().info(f\"received {name}: {{msg}}\")",
                                name = s.name
                            ));
                            setup_lines.push(format!(
                                "    {name}_subscriber.start(on_{name}_message)",
                                name = s.name
                            ));
                            setup_lines.push(format!("    nodes.append({}_subscriber)", s.name));
                        }
                    }
                }
                Interface::Event(_e) => {}
            }
        }
    }

    loop_lines.push("    executor = MultiThreadedExecutor()".to_string());
    loop_lines.push("    for node in nodes:".to_string());
    loop_lines.push("        executor.add_node(node)".to_string());
    loop_lines.push("    try:".to_string());
    loop_lines.push("        executor.spin()".to_string());
    loop_lines.push("    finally:".to_string());
    loop_lines.push("        for timer in timers:".to_string());
    loop_lines.push("            timer.cancel()".to_string());
    loop_lines.push("        executor.shutdown()".to_string());

    teardown_lines.push("        for node in nodes:".to_string());
    teardown_lines.push("            node.destroy_node()".to_string());
    teardown_lines.push("        if rclpy.ok():".to_string());
    teardown_lines.push("            rclpy.shutdown()".to_string());

    let mut out = String::new();
    out.push_str("# Generated by ridlc. Edit this file to host multiple interfaces in one process.\n\n");
    out.push_str("import os\n\n");
    out.push_str("import grpc\n");
    out.push_str("import rclpy\n");
    out.push_str("from rclpy.executors import MultiThreadedExecutor\n\n");
    for line in import_lines {
        out.push_str(&line);
        out.push('\n');
    }
    out.push_str("from robonix_runtime_pb2_grpc import RobonixRuntimeStub\n\n\n");
    out.push_str("def main() -> None:\n");
    out.push_str("    endpoint = os.environ.get(\"ROBONIX_RUNTIME_ENDPOINT\", \"127.0.0.1:50051\")\n");
    out.push_str("    grpc_channel = grpc.insecure_channel(endpoint)\n");
    out.push_str("    runtime_client = RobonixRuntimeStub(grpc_channel)\n");
    out.push_str("    nodes = []\n");
    out.push_str("    timers = []\n\n");
    for line in setup_lines {
        out.push_str(&line);
        out.push('\n');
    }
    out.push('\n');
    for line in loop_lines {
        out.push_str(&line);
        out.push('\n');
    }
    for line in teardown_lines {
        out.push_str(&line);
        out.push('\n');
    }
    out.push_str("\n\nif __name__ == \"__main__\":\n");
    out.push_str("    main()\n");
    fs::write(py_root.join(format!("{}.py", module_name)), out)?;
    Ok(module_name.to_string())
}

pub fn emit_app_skeleton(workspace_src: &Path, files_by_ns: &BTreeMap<String, File>) -> Result<()> {
    let app_root = workspace_src.join(APP_DIR_NAME).join(APP_PACKAGE_NAME);
    if app_root.exists() {
        fs::remove_dir_all(&app_root)?;
    }
    fs::create_dir_all(&app_root)?;

    let py_root = app_root.join(APP_PACKAGE_NAME);
    fs::create_dir_all(&py_root)?;
    fs::write(
        py_root.join("__init__.py"),
        "# User-editable app skeletons generated by ridlc.\n",
    )?;

    let mut entry_points = Vec::new();
    for (namespace, ast) in files_by_ns {
        let ns_pkg_dir = ensure_python_package_tree(&py_root, namespace)?;
        for iface in &ast.interfaces {
            match iface {
                Interface::Query(q) => {
                    let module_name = emit_query_app_module(&ns_pkg_dir, namespace, q)?;
                    entry_points.push((
                        app_entry_name(namespace, &q.name, "server"),
                        format!(
                            "{}.{}.{}:main",
                            APP_PACKAGE_NAME,
                            namespace.replace('/', "."),
                            module_name
                        ),
                    ));
                }
                Interface::Command(c) => {
                    let module_name = emit_command_app_module(&ns_pkg_dir, namespace, c)?;
                    entry_points.push((
                        app_entry_name(namespace, &c.name, "server"),
                        format!(
                            "{}.{}.{}:main",
                            APP_PACKAGE_NAME,
                            namespace.replace('/', "."),
                            module_name
                        ),
                    ));
                }
                Interface::Stream(s) => {
                    let (module_name, role) = emit_stream_app_module(&ns_pkg_dir, namespace, s)?;
                    entry_points.push((
                        app_entry_name(namespace, &s.name, role),
                        format!(
                            "{}.{}.{}:main",
                            APP_PACKAGE_NAME,
                            namespace.replace('/', "."),
                            module_name
                        ),
                    ));
                }
                Interface::Event(e) => {
                    let module_name = emit_event_app_module(&ns_pkg_dir, namespace, e)?;
                    entry_points.push((
                        app_entry_name(namespace, &e.name, "publisher"),
                        format!(
                            "{}.{}.{}:main",
                            APP_PACKAGE_NAME,
                            namespace.replace('/', "."),
                            module_name
                        ),
                    ));
                }
            }
        }
    }

    let combined_module = emit_combined_runtime_module(&py_root, files_by_ns)?;
    entry_points.push((
        "combined_runtime".to_string(),
        format!("{}.{}:main", APP_PACKAGE_NAME, combined_module),
    ));

    emit_app_package_files(&app_root, &entry_points)?;
    Ok(())
}

/// Convert a RIDL type ref like `std_msgs/msg/String` or `robonix_msg/msg/Object[]`
/// into the field syntax expected by ROS `.msg` / `.srv` / `.action` files: `std_msgs/String`, `robonix_msgs/Object[]`.
fn rosidl_field_type(type_ref: &str) -> String {
    let trimmed = type_ref.trim();
    let parts: Vec<&str> = trimmed.split('/').collect();
    if parts.len() >= 3 {
        let mut pkg = parts[0];
        // Vendored package is robonix_msgs; RIDL may use robonix_msg
        if pkg == "robonix_msg" {
            pkg = "robonix_msgs";
        }
        format!("{}/{}", pkg, parts[2])
    } else {
        trimmed.to_string()
    }
}

/// If available, vendor selected Robonix message definitions into the
/// generated ROS IDL tree so that tests don't depend on system-installed
/// packages.
fn maybe_vendor_robonix_msgs(ros_out: &Path) -> Result<()> {
    // Best-effort: vendor the checked-in Robonix message bundle when the
    // sibling repository is available.
    let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("robonix-interfaces")
        .join("lib")
        .join("robonix_msg");

    if src_dir.exists() {
        let dst_dir = ros_out.join(ROBONIX_MSGS_PACKAGE_NAME).join("msg");
        fs::create_dir_all(&dst_dir)?;
        for path in sorted_files(&src_dir)? {
            if path.extension().and_then(|ext| ext.to_str()) != Some("msg") {
                continue;
            }
            let dst = dst_dir.join(path.file_name().unwrap());
            fs::copy(&path, &dst)
                .with_context(|| format!("copy {} -> {}", path.display(), dst.display()))?;
        }
    }

    Ok(())
}

fn emit_command_action_idl(ros_out: &Path, ns: &str, c: &CommandDef) -> Result<()> {
    let action_dir = ros_out.join(ROSIDL_PACKAGE_NAME).join("action");
    fs::create_dir_all(&action_dir)?;

    let mut content = String::new();

    // Goal (use command input, if present).
    if let Some(ref inp) = c.input {
        content.push_str(&format!("{} {}\n", rosidl_field_type(&inp.type_ref), inp.name));
    }
    content.push_str("---\n");

    // Result (use command result, if present).
    if let Some(ref res) = c.result {
        content.push_str(&format!("{} {}\n", rosidl_field_type(&res.type_ref), res.name));
    }
    content.push_str("---\n");

    // Feedback (use command output, if present).
    if let Some(ref out_f) = c.output {
        content.push_str(&format!("{} {}\n", rosidl_field_type(&out_f.type_ref), out_f.name));
    }

    let filename = format!("{}.action", rosidl_symbol_name(ns, &c.name));
    let path = action_dir.join(filename);
    fs::write(path, content)?;
    Ok(())
}

fn emit_query_srv_idl(ros_out: &Path, ns: &str, q: &QueryDef) -> Result<()> {
    let srv_dir = ros_out.join(ROSIDL_PACKAGE_NAME).join("srv");
    fs::create_dir_all(&srv_dir)?;

    let mut content = String::new();
    // Request
    content.push_str(&format!("{} {}\n", rosidl_field_type(&q.request.type_ref), q.request.name));
    content.push_str("---\n");
    // Response
    content.push_str(&format!("{} {}\n", rosidl_field_type(&q.response.type_ref), q.response.name));

    let filename = format!("{}.srv", rosidl_symbol_name(ns, &q.name));
    let path = srv_dir.join(filename);
    fs::write(path, content)?;
    Ok(())
}

pub fn generate(ast: &File, out_dir: &Path) -> Result<()> {
    // Build Python package path from RIDL namespace, e.g.:
    //   namespace robonix/prm/localization
    // becomes directory structure:
    //   <out> / robonix / prm / localization
    let ns = ast.namespace_path().unwrap_or("robonix/unknown");
    let parts: Vec<&str> = ns.split('/').collect();

    let mut pkg_dir = out_dir.to_path_buf();
    for (i, part) in parts.iter().enumerate() {
        pkg_dir = pkg_dir.join(part);
        fs::create_dir_all(&pkg_dir)?;
        // For intermediate segments, ensure __init__.py exists so they are real packages.
        if i < parts.len() - 1 {
            let init_path = pkg_dir.join("__init__.py");
            if !init_path.exists() {
                fs::write(&init_path, "# Generated by ridlc. Package init.\n")?;
            }
        }
    }
    let out_pkg_dir = pkg_dir;

    // Root directory for generated ROS IDL (msg/srv/action). Kept separate
    // from Python package tree so a ROS2 package can vendor or move them.
    let rosidl_out = rosidl_root(out_dir);
    // Also vendor selected Robonix messages (e.g. CommandResult) so the
    // generated workspace can build without relying on system-wide packages.
    maybe_vendor_robonix_msgs(&rosidl_out)?;

    let mut init_content = String::from("# Generated by ridlc. Do not edit.\n\n");
    let mut all_names = Vec::new();

    for iface in &ast.interfaces {
        match iface {
            Interface::Query(q) => {
                let (filename, names) = emit_query_python(&out_pkg_dir, q, ast.namespace_path())?;
                // Also generate corresponding .srv IDL for this query.
                emit_query_srv_idl(&rosidl_out, ns, q)?;
                init_content.push_str(&format!("from .{} import {}\n", filename.trim_end_matches(".py"), names.join(", ")));
                all_names.extend(names);
            }
            Interface::Stream(s) => {
                let (filename, names) = emit_stream_python(&out_pkg_dir, s, ast.namespace_path())?;
                init_content.push_str(&format!("from .{} import {}\n", filename.trim_end_matches(".py"), names.join(", ")));
                all_names.extend(names);
            }
            Interface::Command(c) => {
                let (filename, names) = emit_command_python(&out_pkg_dir, c, ast.namespace_path())?;
                // Also generate corresponding .action IDL for this command.
                emit_command_action_idl(&rosidl_out, ns, c)?;
                init_content.push_str(&format!("from .{} import {}\n", filename.trim_end_matches(".py"), names.join(", ")));
                all_names.extend(names);
            }
            Interface::Event(e) => {
                let (filename, names) = emit_event_python(&out_pkg_dir, e)?;
                init_content.push_str(&format!("from .{} import {}\n", filename.trim_end_matches(".py"), names.join(", ")));
                all_names.extend(names);
            }
        }
    }

    if !all_names.is_empty() {
        init_content.push_str(&format!(
            "\n__all__ = [{}]\n",
            all_names
                .iter()
                .map(|n| format!("{:?}", n))
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    // Leaf package __init__.py contains the re-exports for this namespace.
    fs::write(out_pkg_dir.join("__init__.py"), init_content)?;
    Ok(())
}

fn emit_query_python(out_dir: &Path, q: &QueryDef, namespace: Option<&str>) -> Result<(String, Vec<String>)> {
    let (req_import, req_type_name) = ros2_python_type(&q.request.type_ref);
    let (res_import, res_type_name) = ros2_python_type(&q.response.type_ref);
    let filename = format!("{}_query", q.name);
    let ns_literal = namespace.unwrap_or("robonix/unknown");
    let ros_pkg = ROSIDL_PACKAGE_NAME;
    let srv_type_name = rosidl_symbol_name(ns_literal, &q.name);

    let mut out = String::new();
    out.push_str("# Generated by ridlc. Do not edit.\n");
    out.push_str("# Query: ");
    out.push_str(&q.name);
    out.push_str(" (ROS2 service client/server)\n\n");
    out.push_str("import rclpy\n");
    out.push_str("from rclpy.node import Node\n");
    out.push_str(&format!("from {} import {}\n", req_import, req_type_name));
    out.push_str(&format!("from {} import {}\n", res_import, res_type_name));
    out.push_str("\n");
    out.push_str(&format!("# Service type: {} / {}\n", q.request.type_ref, q.response.type_ref));
    out.push_str("\n");
    out.push_str(&format!("def _load_{}_srv_type():\n", q.name));
    out.push_str("    import importlib\n");
    out.push_str(&format!(
        "    return getattr(importlib.import_module(\"{}.srv\"), \"{}\")\n\n",
        ros_pkg, srv_type_name
    ));

    out.push_str(&format!("def {}_call(node, service_name, request, timeout_sec=10.0):\n", q.name));
    out.push_str("    \"\"\"Blocking call to query (ROS2 service client).\"\"\"\n");
    out.push_str("    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy\n");
    out.push_str(&format!("    srv_type = _load_{}_srv_type()\n", q.name));
    out.push_str("    client = node.create_client(\n");
    out.push_str("        srv_type,\n");
    out.push_str("        service_name,\n");
    out.push_str("        qos_profile=QoSProfile(\n");
    out.push_str("            reliability=ReliabilityPolicy.RELIABLE,\n");
    out.push_str("            history=HistoryPolicy.KEEP_LAST,\n");
    out.push_str("            depth=10,\n");
    out.push_str("        ),\n");
    out.push_str("    )\n");
    out.push_str("    if not client.wait_for_service(timeout_sec=timeout_sec):\n");
    out.push_str("        raise RuntimeError(f'Service {service_name!r} not available')\n");
    out.push_str("    future = client.call_async(request)\n");
    out.push_str("    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)\n");
    out.push_str("    return future.result()\n\n\n");
    out.push_str(&format!("class {}Client(Node):\n", pascal(&q.name)));
    out.push_str("    \"\"\"Base ROS2 service client for query '");
    out.push_str(&q.name);
    out.push_str("'. Channel (service) is provided by runtime.\n\n");
    out.push_str("    Subclass this class and override ``call()`` to implement transport-specific logic.\n");
    out.push_str("    \"\"\"\n\n");
    out.push_str("    def __init__(self, service_name: str):\n");
    out.push_str("        super().__init__('ridlc_query_client_' + service_name.replace('/', '_'))\n");
    out.push_str("        self._service_name = service_name\n\n");
    out.push_str("    def call(self, request, timeout_sec=10.0):\n");
    out.push_str("        \"\"\"Perform the query and return a response.\n");
    out.push_str("\n");
    out.push_str("        Default implementation is abstract; override in subclass.\n");
    out.push_str("        \"\"\"\n");
    out.push_str("        raise NotImplementedError('override call() in subclass')\n\n\n");

    out.push_str(&format!("class {}Server(Node):\n", pascal(&q.name)));
    out.push_str("    \"\"\"Base ROS2 server stub for query '");
    out.push_str(&q.name);
    out.push_str("'. Channel (service) is provided by runtime.\n\n");
    out.push_str("    Subclass this class and override ``start()`` to bind handler and start serving.\n");
    out.push_str("    \"\"\"\n\n");
    out.push_str("    def __init__(self, service_name: str):\n");
    out.push_str("        super().__init__('ridlc_query_server_' + service_name.replace('/', '_'))\n");
    out.push_str("        self._service_name = service_name\n\n");
    out.push_str("    def start(self, handler):\n");
    out.push_str("        \"\"\"Bind a handler(request) -> response and start serving.\n");
    out.push_str("\n");
    out.push_str("        Default implementation is abstract; override in subclass.\n");
    out.push_str("        \"\"\"\n");
    out.push_str("        raise NotImplementedError('override start() in subclass')\n\n");

    // Runtime integration: meta API via gRPC (robonix-server). runtime_client must be
    // RobonixRuntimeStub from robonix_runtime_pb2_grpc.
    out.push_str(&format!(
        "def resolve_{}_service(runtime_client, requester_id: str, target: str) -> str:\n",
        q.name
    ));
    out.push_str("    \"\"\"Resolve query service name via robonix-server gRPC meta API.\n");
    out.push_str("    Returns the ROS2 service name to use as client.\n");
    out.push_str("    \"\"\"\n");
    out.push_str("    from robonix_runtime_pb2 import ResolveQueryRequest\n");
    out.push_str(&format!(
        "    req = ResolveQueryRequest(requester_id=requester_id, target=target, namespace=\"{}\", query_name=\"{}\")\n",
        ns_literal, q.name
    ));
    out.push_str("    resp = runtime_client.ResolveQuery(req)\n");
    out.push_str("    return resp.channel_name\n\n");

    out.push_str(&format!(
        "def register_{}_server(runtime_client, node_id: str) -> str:\n",
        q.name
    ));
    out.push_str("    \"\"\"Register this node as query provider via robonix-server gRPC meta API. Returns assigned ROS2 service name.\"\"\"\n");
    out.push_str("    from robonix_runtime_pb2 import RegisterQueryRequest\n");
    out.push_str(&format!(
        "    req = RegisterQueryRequest(node_id=node_id, namespace=\"{}\", query_name=\"{}\")\n",
        ns_literal, q.name
    ));
    out.push_str("    resp = runtime_client.RegisterQuery(req)\n");
    out.push_str("    return resp.channel_name\n\n");

    out.push_str(&format!("class Ros2{}Server({}Server):\n", pascal(&q.name), pascal(&q.name)));
    out.push_str("    \"\"\"Concrete ROS2 query server with automatic runtime registration.\"\"\"\n\n");
    out.push_str("    def __init__(self, runtime_client, node_id: str):\n");
    out.push_str(&format!("        service_name = register_{}_server(runtime_client, node_id)\n", q.name));
    out.push_str("        super().__init__(service_name)\n");
    out.push_str("        self._runtime_client = runtime_client\n");
    out.push_str("        self._node_id = node_id\n");
    out.push_str(&format!("        self._srv_type = _load_{}_srv_type()\n", q.name));
    out.push_str("        self._service = None\n\n");
    out.push_str("    def start(self, handler):\n");
    out.push_str("        if self._service is None:\n");
    out.push_str("            self._service = self.create_service(self._srv_type, self._service_name, handler)\n");
    out.push_str("        return self._service\n\n");

    out.push_str(&format!("class Ros2{}Client({}Client):\n", pascal(&q.name), pascal(&q.name)));
    out.push_str("    \"\"\"Concrete ROS2 query client with automatic runtime resolution.\"\"\"\n\n");
    out.push_str("    def __init__(self, runtime_client, requester_id: str, target: str):\n");
    out.push_str(&format!("        service_name = resolve_{}_service(runtime_client, requester_id, target)\n", q.name));
    out.push_str("        super().__init__(service_name)\n");
    out.push_str("        self._runtime_client = runtime_client\n");
    out.push_str("        self._requester_id = requester_id\n");
    out.push_str("        self._target = target\n");
    out.push_str(&format!("        self._srv_type = _load_{}_srv_type()\n", q.name));
    out.push_str("        self._client = self.create_client(self._srv_type, self._service_name)\n\n");
    out.push_str("    def call(self, request, timeout_sec=10.0):\n");
    out.push_str("        if not self._client.wait_for_service(timeout_sec=timeout_sec):\n");
    out.push_str("            raise RuntimeError(f'Service {self._service_name!r} not available')\n");
    out.push_str("        future = self._client.call_async(request)\n");
    out.push_str("        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)\n");
    out.push_str("        return future.result()\n\n");

    out.push_str(&format!("def create_{}_server(runtime_client, node_id: str, init_rclpy: bool = True):\n", q.name));
    out.push_str("    if init_rclpy and not rclpy.ok():\n");
    out.push_str("        rclpy.init()\n");
    out.push_str(&format!("    return Ros2{}Server(runtime_client, node_id)\n\n", pascal(&q.name)));

    out.push_str(&format!("def create_{}_client(runtime_client, requester_id: str, target: str, init_rclpy: bool = True):\n", q.name));
    out.push_str("    if init_rclpy and not rclpy.ok():\n");
    out.push_str("        rclpy.init()\n");
    out.push_str(&format!("    return Ros2{}Client(runtime_client, requester_id, target)\n\n", pascal(&q.name)));

    let path = out_dir.join(format!("{}.py", filename));
    fs::write(&path, out)?;
    Ok((
        filename,
        vec![
            format!("{}_call", q.name),
            format!("{}Client", pascal(&q.name)),
            format!("{}Server", pascal(&q.name)),
            format!("Ros2{}Client", pascal(&q.name)),
            format!("Ros2{}Server", pascal(&q.name)),
            format!("create_{}_client", q.name),
            format!("create_{}_server", q.name),
        ],
    ))
}

fn emit_stream_python(out_dir: &Path, s: &StreamDef, namespace: Option<&str>) -> Result<(String, Vec<String>)> {
    let filename = format!("{}_stream", s.name);
    let ns_literal = namespace.unwrap_or("robonix/unknown");
    if s.fields.len() != 1 {
        bail!(
            "stream '{}' must declare exactly one input/output field, found {}",
            s.name,
            s.fields.len()
        );
    }
    let first_field = s.fields.first().unwrap();
    let direction = &first_field.direction;
    let msg_type_ref = first_field.type_ref.as_str();
    let (msg_import, msg_type_name) = ros2_python_type(msg_type_ref);
    let mut out = String::new();
    out.push_str("# Generated by ridlc. Do not edit.\n");
    out.push_str("# Stream: ");
    out.push_str(&s.name);
    out.push_str(match direction {
        StreamDirection::Output => " (ROS2 publisher)\n\n",
        StreamDirection::Input => " (ROS2 subscriber)\n\n",
    });
    out.push_str("import rclpy\n");
    out.push_str("from rclpy.node import Node\n");
    out.push_str("from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy\n\n");
    for field in &s.fields {
        let direction_label = match field.direction {
            StreamDirection::Input => "input",
            StreamDirection::Output => "output",
        };
        out.push_str(&format!("# {} {}: {}\n", direction_label, field.name, field.type_ref));
    }
    out.push_str("\n\n");
    match direction {
        StreamDirection::Output => {
            out.push_str(&format!("class {}Publisher(Node):\n", pascal(&s.name)));
            out.push_str("    \"\"\"Base ROS2 publisher for stream '");
            out.push_str(&s.name);
            out.push_str("'. Channel (topic) provided by runtime.\n\n");
            out.push_str("    Subclass this class and override ``publish()`` to connect to a concrete transport.\n");
            out.push_str("    \"\"\"\n\n");
            out.push_str("    def __init__(self, topic_name: str, msg_type=None):\n");
            out.push_str("        super().__init__('ridlc_stream_pub_' + topic_name.replace('/', '_'))\n");
            out.push_str("        self._topic = topic_name\n");
            out.push_str("        self._msg_type = msg_type\n");
            out.push_str("        self._pub = None\n\n");
            out.push_str("    def publish(self, msg):\n");
            out.push_str("        \"\"\"Publish a message to the stream.\n");
            out.push_str("\n");
            out.push_str("        Default implementation is abstract; override in subclass.\n");
            out.push_str("        \"\"\"\n");
            out.push_str("        raise NotImplementedError('override publish() in subclass')\n\n\n");

            out.push_str(&format!(
                "def register_{}_provider(runtime_client, node_id: str) -> str:\n",
                s.name
            ));
            out.push_str("    \"\"\"Register this node as stream provider via robonix-server gRPC meta API. Returns assigned ROS2 topic name.\"\"\"\n");
            out.push_str("    from robonix_runtime_pb2 import RegisterStreamRequest\n");
            out.push_str(&format!(
                "    req = RegisterStreamRequest(node_id=node_id, namespace=\"{}\", stream_name=\"{}\", role=\"provider\")\n",
                ns_literal, s.name
            ));
            out.push_str("    resp = runtime_client.RegisterStream(req)\n");
            out.push_str("    return resp.channel_name\n\n");

            // Consumer of output stream: resolve topic and subscribe
            out.push_str(&format!(
                "def resolve_{}_consumer_topic(runtime_client, requester_id: str, target: str) -> str:\n",
                s.name
            ));
            out.push_str("    \"\"\"Resolve stream consumer topic via robonix-server gRPC meta API. Returns ROS2 topic name to subscribe to.\"\"\"\n");
            out.push_str("    from robonix_runtime_pb2 import ResolveStreamRequest\n");
            out.push_str(&format!(
                "    req = ResolveStreamRequest(requester_id=requester_id, target=target, namespace=\"{}\", stream_name=\"{}\", role=\"consumer\")\n",
                ns_literal, s.name
            ));
            out.push_str("    resp = runtime_client.ResolveStream(req)\n");
            out.push_str("    return resp.channel_name\n\n");

            out.push_str(&format!("class {}Subscriber(Node):\n", pascal(&s.name)));
            out.push_str("    \"\"\"Base ROS2 subscriber for stream '");
            out.push_str(&s.name);
            out.push_str("'. Channel (topic) provided by runtime.\n\n");
            out.push_str("    Subclass this class and override ``start()`` to connect to a concrete transport.\n");
            out.push_str("    \"\"\"\n\n");
            out.push_str("    def __init__(self, topic_name: str, msg_type=None):\n");
            out.push_str("        super().__init__('ridlc_stream_sub_' + topic_name.replace('/', '_'))\n");
            out.push_str("        self._topic = topic_name\n");
            out.push_str("        self._msg_type = msg_type\n");
            out.push_str("        self._sub = None\n\n");
            out.push_str("    def start(self, callback):\n");
            out.push_str("        \"\"\"Start subscribing and dispatch messages to callback(msg).\n");
            out.push_str("\n");
            out.push_str("        Default implementation is abstract; override in subclass.\n");
            out.push_str("        \"\"\"\n");
            out.push_str("        raise NotImplementedError('override start() in subclass')\n\n");
        }
        StreamDirection::Input => {
            out.push_str(&format!("class {}Subscriber(Node):\n", pascal(&s.name)));
            out.push_str("    \"\"\"Base ROS2 subscriber stub for stream '");
            out.push_str(&s.name);
            out.push_str("'. Channel (topic) provided by runtime.\n\n");
            out.push_str("    Subclass this class and override ``start()`` to connect to a concrete transport.\n");
            out.push_str("    \"\"\"\n\n");
            out.push_str("    def __init__(self, topic_name: str, msg_type=None):\n");
            out.push_str("        super().__init__('ridlc_stream_sub_' + topic_name.replace('/', '_'))\n");
            out.push_str("        self._topic = topic_name\n");
            out.push_str("        self._msg_type = msg_type\n");
            out.push_str("        self._sub = None\n\n");
            out.push_str("    def start(self, callback):\n");
            out.push_str("        \"\"\"Start subscribing and dispatch messages to callback(msg).\n");
            out.push_str("\n");
            out.push_str("        Default implementation is abstract; override in subclass.\n");
            out.push_str("        \"\"\"\n");
            out.push_str("        raise NotImplementedError('override start() in subclass')\n\n");

            out.push_str(&format!(
                "def resolve_{}_consumer_topic(runtime_client, requester_id: str, target: str) -> str:\n",
                s.name
            ));
            out.push_str("    \"\"\"Resolve stream consumer topic via robonix-server gRPC meta API. Returns ROS2 topic name to subscribe to.\"\"\"\n");
            out.push_str("    from robonix_runtime_pb2 import ResolveStreamRequest\n");
            out.push_str(&format!(
                "    req = ResolveStreamRequest(requester_id=requester_id, target=target, namespace=\"{}\", stream_name=\"{}\", role=\"consumer\")\n",
                ns_literal, s.name
            ));
            out.push_str("    resp = runtime_client.ResolveStream(req)\n");
            out.push_str("    return resp.channel_name\n\n");
        }
    }

    out.push_str(&format!("def _load_{}_msg_type():\n", s.name));
    out.push_str(&format!("    from {} import {}\n", msg_import, msg_type_name));
    out.push_str(&format!("    return {}\n\n", msg_type_name));

    let exported_names = match direction {
        StreamDirection::Output => {
            out.push_str(&format!("class Ros2{}Publisher({}Publisher):\n", pascal(&s.name), pascal(&s.name)));
            out.push_str("    \"\"\"Concrete ROS2 stream publisher with automatic runtime registration.\"\"\"\n\n");
            out.push_str("    def __init__(self, runtime_client, node_id: str):\n");
            out.push_str(&format!("        topic_name = register_{}_provider(runtime_client, node_id)\n", s.name));
            out.push_str(&format!("        msg_type = _load_{}_msg_type()\n", s.name));
            out.push_str("        super().__init__(topic_name, msg_type)\n");
            out.push_str("        self._runtime_client = runtime_client\n");
            out.push_str("        self._node_id = node_id\n");
            out.push_str("        self._pub = self.create_publisher(self._msg_type, self._topic, QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10))\n\n");
            out.push_str("    def publish(self, msg):\n");
            out.push_str("        self._pub.publish(msg)\n\n");

            out.push_str(&format!("def create_{}_publisher(runtime_client, node_id: str, init_rclpy: bool = True):\n", s.name));
            out.push_str("    if init_rclpy and not rclpy.ok():\n");
            out.push_str("        rclpy.init()\n");
            out.push_str(&format!("    return Ros2{}Publisher(runtime_client, node_id)\n\n", pascal(&s.name)));

            out.push_str(&format!("class Ros2{}Subscriber({}Subscriber):\n", pascal(&s.name), pascal(&s.name)));
            out.push_str("    \"\"\"Concrete ROS2 stream subscriber with automatic runtime resolution.\"\"\"\n\n");
            out.push_str("    def __init__(self, runtime_client, requester_id: str, target: str):\n");
            out.push_str(&format!("        topic_name = resolve_{}_consumer_topic(runtime_client, requester_id, target)\n", s.name));
            out.push_str(&format!("        msg_type = _load_{}_msg_type()\n", s.name));
            out.push_str("        super().__init__(topic_name, msg_type)\n");
            out.push_str("        self._runtime_client = runtime_client\n");
            out.push_str("        self._requester_id = requester_id\n");
            out.push_str("        self._target = target\n\n");
            out.push_str("    def start(self, callback):\n");
            out.push_str("        if self._sub is None:\n");
            out.push_str("            self._sub = self.create_subscription(self._msg_type, self._topic, callback, QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10))\n");
            out.push_str("        return self._sub\n\n");

            out.push_str(&format!("def create_{}_subscriber(runtime_client, requester_id: str, target: str, init_rclpy: bool = True):\n", s.name));
            out.push_str("    if init_rclpy and not rclpy.ok():\n");
            out.push_str("        rclpy.init()\n");
            out.push_str(&format!("    return Ros2{}Subscriber(runtime_client, requester_id, target)\n\n", pascal(&s.name)));

            vec![
                format!("{}Publisher", pascal(&s.name)),
                format!("Ros2{}Publisher", pascal(&s.name)),
                format!("create_{}_publisher", s.name),
                format!("{}Subscriber", pascal(&s.name)),
                format!("Ros2{}Subscriber", pascal(&s.name)),
                format!("create_{}_subscriber", s.name),
            ]
        }
        StreamDirection::Input => {
            out.push_str(&format!("class Ros2{}Subscriber({}Subscriber):\n", pascal(&s.name), pascal(&s.name)));
            out.push_str("    \"\"\"Concrete ROS2 stream subscriber with automatic runtime resolution.\"\"\"\n\n");
            out.push_str("    def __init__(self, runtime_client, requester_id: str, target: str):\n");
            out.push_str(&format!("        topic_name = resolve_{}_consumer_topic(runtime_client, requester_id, target)\n", s.name));
            out.push_str(&format!("        msg_type = _load_{}_msg_type()\n", s.name));
            out.push_str("        super().__init__(topic_name, msg_type)\n");
            out.push_str("        self._runtime_client = runtime_client\n");
            out.push_str("        self._requester_id = requester_id\n");
            out.push_str("        self._target = target\n\n");
            out.push_str("    def start(self, callback):\n");
            out.push_str("        if self._sub is None:\n");
            out.push_str("            self._sub = self.create_subscription(self._msg_type, self._topic, callback, QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10))\n");
            out.push_str("        return self._sub\n\n");

            out.push_str(&format!("def create_{}_subscriber(runtime_client, requester_id: str, target: str, init_rclpy: bool = True):\n", s.name));
            out.push_str("    if init_rclpy and not rclpy.ok():\n");
            out.push_str("        rclpy.init()\n");
            out.push_str(&format!("    return Ros2{}Subscriber(runtime_client, requester_id, target)\n\n", pascal(&s.name)));

            vec![
                format!("{}Subscriber", pascal(&s.name)),
                format!("Ros2{}Subscriber", pascal(&s.name)),
                format!("create_{}_subscriber", s.name),
            ]
        }
    };

    let path = out_dir.join(format!("{}.py", filename));
    fs::write(&path, out)?;
    Ok((filename, exported_names))
}

fn emit_command_python(out_dir: &Path, c: &CommandDef, namespace: Option<&str>) -> Result<(String, Vec<String>)> {
    let filename = format!("{}_command", c.name);
    let ns_literal = namespace.unwrap_or("robonix/unknown");
    let ros_pkg = ROSIDL_PACKAGE_NAME;
    let action_type_name = rosidl_symbol_name(ns_literal, &c.name);
    let mut out = String::new();
    out.push_str("# Generated by ridlc. Do not edit.\n");
    out.push_str("# Command: ");
    out.push_str(&c.name);
    out.push_str(" (ROS2 action or service server)\n\n");
    out.push_str("import rclpy\n");
    out.push_str("from rclpy.node import Node\n\n");
    if let Some(ref inp) = c.input {
        out.push_str(&format!("# input: {} ({})\n", inp.name, inp.type_ref));
    }
    if let Some(ref out_f) = c.output {
        out.push_str(&format!("# output: {} ({})\n", out_f.name, out_f.type_ref));
    }
    if let Some(ref res) = c.result {
        out.push_str(&format!("# result: {} ({})\n", res.name, res.type_ref));
    }
    out.push_str("\n\n");
    out.push_str(&format!("class {}Server(Node):\n", pascal(&c.name)));
    out.push_str("    \"\"\"Base ROS2 server stub for command '");
    out.push_str(&c.name);
    out.push_str("'. Channel (action or service) is provided by runtime.\n\n");
    out.push_str("    Subclass this class and override ``execute()`` to implement the command.\n");
    out.push_str("    \"\"\"\n\n");
    out.push_str("    def __init__(self, service_or_action_name: str):\n");
    out.push_str("        super().__init__('ridlc_command_server_' + service_or_action_name.replace('/', '_'))\n");
    out.push_str("        self._name = service_or_action_name\n\n");
    out.push_str("    def execute(self, request, goal_handle=None):\n");
    out.push_str("        \"\"\"Handle a single command request.\n");
    out.push_str("\n");
    out.push_str("        goal_handle: ROS2 action goal handle, for publishing feedback via goal_handle.publish_feedback().\n");
    out.push_str("        Default implementation is abstract; override in subclass.\n");
    out.push_str("        \"\"\"\n");
    out.push_str("        raise NotImplementedError('override execute() in subclass')\n\n\n");

    out.push_str(&format!("class {}Client(Node):\n", pascal(&c.name)));
    out.push_str("    \"\"\"Base ROS2 client stub for command '");
    out.push_str(&c.name);
    out.push_str("'. Channel (action or service) is provided by runtime.\n\n");
    out.push_str("    Subclass this class and override ``send()`` to connect to a concrete transport.\n");
    out.push_str("    \"\"\"\n\n");
    out.push_str("    def __init__(self, service_or_action_name: str):\n");
    out.push_str("        super().__init__('ridlc_command_client_' + service_or_action_name.replace('/', '_'))\n");
    out.push_str("        self._name = service_or_action_name\n\n");
    out.push_str("    def send(self, request):\n");
    out.push_str("        \"\"\"Send a command request.\n");
    out.push_str("\n");
    out.push_str("        Default implementation is abstract; override in subclass.\n");
    out.push_str("        \"\"\"\n");
    out.push_str("        raise NotImplementedError('override send() in subclass')\n\n");

    // Runtime integration: meta API via gRPC. runtime_client = RobonixRuntimeStub (robonix_runtime_pb2_grpc).
    out.push_str(&format!(
        "def register_{}_server(runtime_client, node_id: str) -> str:\n",
        c.name
    ));
    out.push_str("    \"\"\"Register this node as command provider via robonix-server gRPC meta API. Returns assigned ROS2 action name.\"\"\"\n");
    out.push_str("    from robonix_runtime_pb2 import RegisterCommandRequest\n");
    out.push_str(&format!(
        "    req = RegisterCommandRequest(node_id=node_id, namespace=\"{}\", command_name=\"{}\")\n",
        ns_literal, c.name
    ));
    out.push_str("    resp = runtime_client.RegisterCommand(req)\n");
    out.push_str("    return resp.channel_name\n\n");

    out.push_str(&format!(
        "def resolve_{}_client_action(runtime_client, requester_id: str, target: str) -> str:\n",
        c.name
    ));
    out.push_str("    \"\"\"Resolve command action name via robonix-server gRPC meta API. Returns ROS2 action name to use as client.\"\"\"\n");
    out.push_str("    from robonix_runtime_pb2 import ResolveCommandRequest\n");
    out.push_str(&format!(
        "    req = ResolveCommandRequest(requester_id=requester_id, target=target, namespace=\"{}\", command_name=\"{}\")\n",
        ns_literal, c.name
    ));
    out.push_str("    resp = runtime_client.ResolveCommand(req)\n");
    out.push_str("    return resp.channel_name\n\n");

    out.push_str(&format!("def _load_{}_action_type():\n", c.name));
    out.push_str("    import importlib\n");
    out.push_str(&format!(
        "    return getattr(importlib.import_module(\"{}.action\"), \"{}\")\n\n",
        ros_pkg, action_type_name
    ));

    out.push_str(&format!("class Ros2{}Server({}Server):\n", pascal(&c.name), pascal(&c.name)));
    out.push_str("    \"\"\"Concrete ROS2 action server with automatic runtime registration.\"\"\"\n\n");
    out.push_str("    def __init__(self, runtime_client, node_id: str):\n");
    out.push_str(&format!("        action_name = register_{}_server(runtime_client, node_id)\n", c.name));
    out.push_str("        super().__init__(action_name)\n");
    out.push_str("        self._runtime_client = runtime_client\n");
    out.push_str("        self._node_id = node_id\n");
    out.push_str(&format!("        self._action_type = _load_{}_action_type()\n", c.name));
    out.push_str("        self._action_server = None\n\n");
    out.push_str("    def start(self):\n");
    out.push_str("        from rclpy.action import ActionServer\n");
    out.push_str("        if self._action_server is None:\n");
    out.push_str("            self._action_server = ActionServer(self, self._action_type, self._name, execute_callback=self._execute_callback)\n");
    out.push_str("        return self._action_server\n\n");
    out.push_str("    async def _execute_callback(self, goal_handle):\n");
    out.push_str("        result = self.execute(goal_handle.request, goal_handle)\n");
    out.push_str("        goal_handle.succeed()\n");
    out.push_str("        return result\n\n");

    out.push_str(&format!("class Ros2{}Client({}Client):\n", pascal(&c.name), pascal(&c.name)));
    out.push_str("    \"\"\"Concrete ROS2 action client with automatic runtime resolution.\"\"\"\n\n");
    out.push_str("    def __init__(self, runtime_client, requester_id: str, target: str):\n");
    out.push_str(&format!("        action_name = resolve_{}_client_action(runtime_client, requester_id, target)\n", c.name));
    out.push_str("        super().__init__(action_name)\n");
    out.push_str("        self._runtime_client = runtime_client\n");
    out.push_str("        self._requester_id = requester_id\n");
    out.push_str("        self._target = target\n");
    out.push_str(&format!("        self._action_type = _load_{}_action_type()\n", c.name));
    out.push_str("        from rclpy.action import ActionClient\n");
    out.push_str("        self._client = ActionClient(self, self._action_type, self._name)\n\n");
    out.push_str("    def send(self, request, timeout_sec=10.0):\n");
    out.push_str("        if not self._client.wait_for_server(timeout_sec=timeout_sec):\n");
    out.push_str("            raise RuntimeError(f'Action {self._name!r} not available')\n");
    out.push_str("        future = self._client.send_goal_async(request)\n");
    out.push_str("        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)\n");
    out.push_str("        return future.result()\n\n");

    out.push_str(&format!("def create_{}_server(runtime_client, node_id: str, init_rclpy: bool = True):\n", c.name));
    out.push_str("    if init_rclpy and not rclpy.ok():\n");
    out.push_str("        rclpy.init()\n");
    out.push_str(&format!("    return Ros2{}Server(runtime_client, node_id)\n\n", pascal(&c.name)));

    out.push_str(&format!("def create_{}_client(runtime_client, requester_id: str, target: str, init_rclpy: bool = True):\n", c.name));
    out.push_str("    if init_rclpy and not rclpy.ok():\n");
    out.push_str("        rclpy.init()\n");
    out.push_str(&format!("    return Ros2{}Client(runtime_client, requester_id, target)\n\n", pascal(&c.name)));

    let path = out_dir.join(format!("{}.py", filename));
    fs::write(&path, out)?;
    Ok((
        filename,
        vec![
            format!("{}Server", pascal(&c.name)),
            format!("{}Client", pascal(&c.name)),
            format!("Ros2{}Server", pascal(&c.name)),
            format!("Ros2{}Client", pascal(&c.name)),
            format!("create_{}_server", c.name),
            format!("create_{}_client", c.name),
        ],
    ))
}

fn emit_event_python(out_dir: &Path, e: &EventDef) -> Result<(String, Vec<String>)> {
    let filename = format!("{}_event", e.name);
    let mut out = String::new();
    out.push_str("# Generated by ridlc. Do not edit.\n");
    out.push_str("# Event: ");
    out.push_str(&e.name);
    out.push_str(" (ROS2 publisher, one-shot)\n\n");
    out.push_str("import rclpy\n");
    out.push_str("from rclpy.node import Node\n");
    out.push_str("from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy\n\n");
    out.push_str(&format!("# payload: {} ({})\n", e.payload.name, e.payload.type_ref));
    out.push_str("\n\n");
    out.push_str(&format!("class {}Publisher(Node):\n", pascal(&e.name)));
    out.push_str("    \"\"\"Generated ROS2 publisher for event '");
    out.push_str(&e.name);
    out.push_str("'.\"\"\"\n\n");
    out.push_str("    def __init__(self, topic_name: str, msg_type=None):\n");
    out.push_str("        super().__init__('ridlc_event_pub_' + topic_name.replace('/', '_'))\n");
    out.push_str("        self._topic = topic_name\n");
    out.push_str("        self._msg_type = msg_type\n");
    out.push_str("        self._pub = None\n\n");
    out.push_str("    def emit(self, msg):\n");
    out.push_str("        if self._pub is None and self._msg_type is not None:\n");
    out.push_str("            self._pub = self.create_publisher(\n");
    out.push_str("                self._msg_type,\n");
    out.push_str("                self._topic,\n");
    out.push_str("                QoSProfile(depth=1),\n");
    out.push_str("            )\n");
    out.push_str("        if self._pub is not None:\n");
    out.push_str("            self._pub.publish(msg)\n\n");
    let path = out_dir.join(format!("{}.py", filename));
    fs::write(&path, out)?;
    Ok((
        filename,
        vec![format!("{}Publisher", pascal(&e.name))],
    ))
}

fn pascal(s: &str) -> String {
    let mut out = String::new();
    let mut cap = true;
    for c in s.chars() {
        if c == '_' || c == ' ' {
            cap = true;
        } else if cap {
            out.extend(c.to_uppercase());
            cap = false;
        } else {
            out.push(c);
        }
    }
    out
}
