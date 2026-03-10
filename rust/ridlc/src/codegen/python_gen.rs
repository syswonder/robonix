// SPDX-License-Identifier: MulanPSL-2.0
// Python ROS2 code generator for RIDL (PoC)
// Generates rclpy-based client/server stubs for query, stream, command, event.

use anyhow::{Context, Result};
use std::fs;
use std::path::{Path, PathBuf};

use crate::ast::{CommandDef, EventDef, File, Interface, QueryDef, StreamDef};

/// Path to proto/gen (gRPC Python stubs) relative to ridlc crate root.
fn proto_gen_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("proto").join("gen")
}

/// Copy gRPC runtime client modules into out_dir so generated code works out of the box.
/// Call once per --lang python run. If proto/gen is missing, log and skip (user must run gen_grpc.sh).
pub fn emit_runtime_grpc(out_dir: &Path) -> Result<()> {
    let gen_dir = proto_gen_dir();
    let files = ["robonix_runtime_pb2.py", "robonix_runtime_pb2_grpc.py"];
    for f in &files {
        let src = gen_dir.join(f);
        if !src.exists() {
            eprintln!(
                "[ridlc] warning: {} not found (run proto/gen_grpc.sh in ridlc repo); runtime helpers will fail at import",
                src.display()
            );
            return Ok(());
        }
        let dst = out_dir.join(f);
        fs::copy(&src, &dst).with_context(|| format!("copy {} -> {}", src.display(), dst.display()))?;
    }
    eprintln!("[ridlc] emitted gRPC runtime client into {}", out_dir.display());
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
  <maintainer email="user@todo.todo">robonix</maintainer>
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
    maintainer_email='user@todo.todo',
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
    eprintln!("[ridlc] emitted ROS2 package files (package.xml, setup.cfg, setup.py) for '{}'", package_name);
    Ok(())
}

/// Map RIDL type ref (e.g. "geometry_msgs/msg/Twist") to ROS2 Python import and type name.
fn ros2_python_type(type_ref: &str) -> (String, String) {
    let trimmed = type_ref.trim();
    let parts: Vec<&str> = trimmed.split('/').collect();
    if parts.len() >= 3 {
        (format!("{}.{}", parts[0], parts[1]), parts[2].to_string())
    } else {
        ("std_msgs.msg".to_string(), trimmed.to_string())
    }
}

fn python_package_from_namespace(_ns: Option<&str>) -> String {
    _ns.unwrap_or("robonix.unknown")
        .replace('/', ".")
        .to_lowercase()
}

const ROSIDL_PACKAGE_NAME: &str = "robonix_interfaces_ros2";

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

fn emit_command_action_idl(ros_out: &Path, ns: &str, c: &CommandDef) -> Result<()> {
    let action_dir = ros_out.join(ROSIDL_PACKAGE_NAME).join("action");
    fs::create_dir_all(&action_dir)?;

    let mut content = String::new();

    // Goal (use command input, if present).
    if let Some(ref inp) = c.input {
        content.push_str(&format!("{} {}\n", inp.type_ref, inp.name));
    }
    content.push_str("---\n");

    // Result (use command result, if present).
    if let Some(ref res) = c.result {
        content.push_str(&format!("{} {}\n", res.type_ref, res.name));
    }
    content.push_str("---\n");

    // Feedback (use command output, if present).
    if let Some(ref out_f) = c.output {
        content.push_str(&format!("{} {}\n", out_f.type_ref, out_f.name));
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
    content.push_str(&format!("{} {}\n", q.request.type_ref, q.request.name));
    content.push_str("---\n");
    // Response
    content.push_str(&format!("{} {}\n", q.response.type_ref, q.response.name));

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
    out.push_str("    from rclpy.qos import QoSProfile, ReliabilityProfile, HistoryPolicy\n");
    out.push_str(&format!("    srv_type = _load_{}_srv_type()\n", q.name));
    out.push_str("    client = node.create_client(\n");
    out.push_str("        srv_type,\n");
    out.push_str("        service_name,\n");
    out.push_str("        qos_profile=QoSProfile(\n");
    out.push_str("            reliability=ReliabilityProfile.RELIABLE,\n");
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
    let msg_type_ref = s.outputs.first().map(|o| o.type_ref.as_str()).unwrap_or("std_msgs/msg/String");
    let (msg_import, msg_type_name) = ros2_python_type(msg_type_ref);
    let mut out = String::new();
    out.push_str("# Generated by ridlc. Do not edit.\n");
    out.push_str("# Stream: ");
    out.push_str(&s.name);
    out.push_str(" (ROS2 publisher)\n\n");
    out.push_str("import rclpy\n");
    out.push_str("from rclpy.node import Node\n");
    out.push_str("from rclpy.qos import QoSProfile, ReliabilityProfile, HistoryPolicy\n\n");
    for o in &s.outputs {
        out.push_str(&format!("# output {}: {}\n", o.name, o.type_ref));
    }
    out.push_str("\n\n");
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

    // Runtime integration: meta API via gRPC. runtime_client = RobonixRuntimeStub (robonix_runtime_pb2_grpc).
    let ns_literal = namespace.unwrap_or("<unknown-namespace>");
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

    out.push_str(&format!("def _load_{}_msg_type():\n", s.name));
    out.push_str(&format!("    from {} import {}\n", msg_import, msg_type_name));
    out.push_str(&format!("    return {}\n\n", msg_type_name));

    out.push_str(&format!("class Ros2{}Publisher({}Publisher):\n", pascal(&s.name), pascal(&s.name)));
    out.push_str("    \"\"\"Concrete ROS2 stream publisher with automatic runtime registration.\"\"\"\n\n");
    out.push_str("    def __init__(self, runtime_client, node_id: str):\n");
    out.push_str(&format!("        topic_name = register_{}_provider(runtime_client, node_id)\n", s.name));
    out.push_str(&format!("        msg_type = _load_{}_msg_type()\n", s.name));
    out.push_str("        super().__init__(topic_name, msg_type)\n");
    out.push_str("        self._runtime_client = runtime_client\n");
    out.push_str("        self._node_id = node_id\n");
    out.push_str("        self._pub = self.create_publisher(self._msg_type, self._topic, QoSProfile(reliability=ReliabilityProfile.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10))\n\n");
    out.push_str("    def publish(self, msg):\n");
    out.push_str("        self._pub.publish(msg)\n\n");

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
    out.push_str("            self._sub = self.create_subscription(self._msg_type, self._topic, callback, QoSProfile(reliability=ReliabilityProfile.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10))\n");
    out.push_str("        return self._sub\n\n");

    out.push_str(&format!("def create_{}_publisher(runtime_client, node_id: str, init_rclpy: bool = True):\n", s.name));
    out.push_str("    if init_rclpy and not rclpy.ok():\n");
    out.push_str("        rclpy.init()\n");
    out.push_str(&format!("    return Ros2{}Publisher(runtime_client, node_id)\n\n", pascal(&s.name)));

    out.push_str(&format!("def create_{}_subscriber(runtime_client, requester_id: str, target: str, init_rclpy: bool = True):\n", s.name));
    out.push_str("    if init_rclpy and not rclpy.ok():\n");
    out.push_str("        rclpy.init()\n");
    out.push_str(&format!("    return Ros2{}Subscriber(runtime_client, requester_id, target)\n\n", pascal(&s.name)));

    let path = out_dir.join(format!("{}.py", filename));
    fs::write(&path, out)?;
    Ok((
        filename,
        vec![
            format!("{}Publisher", pascal(&s.name)),
            format!("{}Subscriber", pascal(&s.name)),
            format!("Ros2{}Publisher", pascal(&s.name)),
            format!("Ros2{}Subscriber", pascal(&s.name)),
            format!("create_{}_publisher", s.name),
            format!("create_{}_subscriber", s.name),
        ],
    ))
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
    out.push_str("    def execute(self, request):\n");
    out.push_str("        \"\"\"Handle a single command request.\n");
    out.push_str("\n");
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
    out.push_str("        result = self.execute(goal_handle.request)\n");
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
    out.push_str("from rclpy.qos import QoSProfile, ReliabilityProfile, HistoryPolicy\n\n");
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
