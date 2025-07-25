# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

from enum import Enum
from typing import List, Tuple, Dict, Any
from dataclasses import dataclass

# documentation:
# For input and output fields, the argument and return values are standardized to dict or class.
# - For simple types, use {<str>: <class>} (e.g., {"x": float, "y": float, "z": float})
# - For complex types, define a dataclass/class (e.g., EOS_SPEC_Image) and use the class directly as the value.
# The TYPE here is the actual Python type symbol, like str, int, float, or a dataclass/class.
# For fields other than input and output, the above rules are not applied!
# Example:
#   { "input": None, "output": EOS_SPEC_Image }
#   { "input": {"target": str, "distance": float}, "output": {"success": bool} }
# This enables strict type checking, including for nested/complex structures.
# Also noted that EOS_SPEC is <class> for complex types, not just <Dict>.
# - wheatfox 2025.7.25

# "skill_name": {
#     "description": <str>,
#     "type": <str>
#     "input": <Dict[str, Any]>, // Any is Class or Dict
#     "output": <Dict[str, Any]>,
#     "dependencies": <List[str]>, // List of skill names, use [] for no dependencies
# }


class EOS_TYPE_ImageFormat(Enum):
    JPEG = "jpeg"
    PNG = "png"
    BMP = "bmp"
    TIFF = "tiff"
    WEBP = "webp"


class EOS_TYPE_CameraType(Enum):
    RGB = "rgb"
    DEPTH = "depth"
    INFRARED = "infrared"


class EOS_SkillType(Enum):
    CAPABILITY = "capability"
    SKILL = "skill"


EOS_TYPE_PosXYZ = {"x": float, "y": float, "z": float}


# TODO: add recursive type check in entity
@dataclass
class EOS_TYPE_ImageMetadata:
    width: int
    height: int
    format: EOS_TYPE_ImageFormat
    camera_type: EOS_TYPE_CameraType

    def __str__(self):
        from rich.pretty import pretty_repr

        return pretty_repr(self)


@dataclass
class EOS_TYPE_Image:
    image_raw: bytes
    metadata: EOS_TYPE_ImageMetadata

    def __str__(self):
        from rich.pretty import pretty_repr

        meta_str = pretty_repr(self.metadata)
        if self.image_raw is not None:
            preview = self.image_raw[:8]
            preview_str = f"bytes[{len(self.image_raw)}]: {preview!r}..."
        else:
            preview_str = "None"
        return f"EOS_Image(\n  image_raw={preview_str},\n  metadata={meta_str}\n)"


# --- Additional Data Types for Capabilities ---


@dataclass
class EOS_TYPE_AudioBuffer:
    data: bytes
    sample_rate: int
    channels: int


@dataclass
class EOS_TYPE_DepthFrame:
    data: bytes
    width: int
    height: int
    format: str  # e.g. 'raw', 'png', etc.


@dataclass
class EOS_TYPE_ImagePair:
    rgb_image: "EOS_TYPE_Image"
    depth_image: "EOS_TYPE_DepthFrame"


@dataclass
class EOS_TYPE_BBox:
    x: int
    y: int
    w: int
    h: int


@dataclass
class EOS_TYPE_IMUData:
    accel: Tuple[float, float, float]
    gyro: Tuple[float, float, float]
    mag: Tuple[float, float, float]


@dataclass
class EOS_TYPE_LidarScan:
    ranges: List[float]
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    scan_time: float
    range_min: float
    range_max: float


@dataclass
class EOS_TYPE_Pose2D:
    x: float
    y: float
    theta: float


@dataclass
class EOS_TYPE_Pose6D:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass
class EOS_TYPE_DetectedObject:
    label: str
    bbox: EOS_TYPE_BBox
    confidence: float


@dataclass
class EOS_TYPE_MaskMap:
    mask: bytes  # e.g. PNG mask
    width: int
    height: int


@dataclass
class EOS_TYPE_GridMap:
    data: List[List[int]]
    resolution: float
    origin: EOS_TYPE_Pose2D


@dataclass
class EOS_TYPE_Transform:
    translation: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]  # Quaternion


@dataclass
class EOS_TYPE_Path:
    poses: List[EOS_TYPE_Pose2D]


@dataclass
class EOS_TYPE_Feature:
    name: str
    position: EOS_TYPE_Pose2D


@dataclass
class EOS_TYPE_FeatureList:
    features: List[EOS_TYPE_Feature]


EntityPath = str  # Represents the path of an entity.

EntityPathAndRequired = {
    "entity": EntityPath,
    "required": List[str],
}

EOS_SKILL_SPECS = {
    # naming rules: [c/s]_<category>_<name>
    # c: capability, s: skill
    # dependencies: list of skill that THIS entity should bind before using this skill
    "c_space_getpos": {
        "description": "Get the position of the entity",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": EOS_TYPE_PosXYZ,
        "dependencies": [],
    },
    "c_space_move": {
        "description": "Move the entity to the given position",
        "type": EOS_SkillType.CAPABILITY,
        "input": EOS_TYPE_PosXYZ,
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_image_caputre": {
        # cap_camera_rgb, cap_camera_depth, cap_camera_ir
        "description": "Capture an image, should be implemented on camera or something",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": EOS_TYPE_Image,  # Use the dataclass
        "dependencies": [],
    },
    "s_space_move2entity": {
        "description": "Move the entity to vicinity of another entity",
        "type": EOS_SkillType.SKILL,
        "input": {
            # TODO: use list to support multiple options for passing target_entity (with different Types)
            "target_entity": [
                EntityPathAndRequired,
                EntityPath,
            ],  # Use EntityPath (str)，and EntityPathAndRequired is a wrapper of EntityPath (str) with required skills, or just EntityPath (str) without skill checks
            # e.g. robot1.s_space_move2entity(target_entity={entity="/room1/book1",required="c_space_getpos"}, distance=0.5)
            # e.g. robot1.s_space_move2entity(target_entity="/room1/book1", distance=0.5)
            "distance": float,
        },
        "output": {"success": bool},
        "dependencies": ["c_space_move", "c_space_getpos"],
    },
    # --- Image Perception Capabilities ---
    "c_image_camera_rgb": {
        "description": "Capture one RGB image frame from camera.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"camera_name": str, "timeout_sec": float},
        "output": EOS_TYPE_Image,
        "dependencies": [],
    },
    "c_image_camera_rgb_depth": {
        "description": "Capture RGB and Depth images at the same timestamp.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"camera_name": str, "timeout_sec": float},
        "output": EOS_TYPE_ImagePair,
        "dependencies": [],
    },
    "c_image_camera_ir": {
        "description": "Capture one IR image frame from camera.",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": EOS_TYPE_Image,
        "dependencies": [],
    },
    "c_image_resize": {
        "description": "Resize image to given size.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"img": EOS_TYPE_Image, "size": Tuple[int, int]},
        "output": EOS_TYPE_Image,
        "dependencies": [],
    },
    "c_image_crop": {
        "description": "Crop image to bounding box.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"img": EOS_TYPE_Image, "box": EOS_TYPE_BBox},
        "output": EOS_TYPE_Image,
        "dependencies": [],
    },
    "c_image_convert_format": {
        "description": "Convert image format.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"img": EOS_TYPE_Image, "fmt": str},
        "output": EOS_TYPE_Image,
        "dependencies": [],
    },
    # --- Audio Capabilities ---
    "c_audio_record": {
        "description": "Record audio for a given duration.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"duration": int},
        "output": EOS_TYPE_AudioBuffer,
        "dependencies": [],
    },
    "c_audio_play": {
        "description": "Play audio buffer.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"audio": EOS_TYPE_AudioBuffer},
        "output": None,
        "dependencies": [],
    },
    "c_audio_resample": {
        "description": "Resample audio to a new rate.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"audio": EOS_TYPE_AudioBuffer, "rate": int},
        "output": EOS_TYPE_AudioBuffer,
        "dependencies": [],
    },
    "c_asr_infer": {
        "description": "ASR inference from audio.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"audio": EOS_TYPE_AudioBuffer},
        "output": {"text": str},
        "dependencies": [],
    },
    "c_tts_synthesize": {
        "description": "TTS synthesis from text.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"text": str},
        "output": EOS_TYPE_AudioBuffer,
        "dependencies": [],
    },
    "c_audio_record_to_file": {
        "description": "Record audio and save to file.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"duration": int, "path": str},
        "output": {"path": str},
        "dependencies": [],
    },
    "c_audio_play_file": {
        "description": "Play audio from file.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"path": str},
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_asr_from_file": {
        "description": "ASR inference from audio file.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"path": str},
        "output": {"text": str},
        "dependencies": [],
    },
    "c_tts_to_file": {
        "description": "TTS synthesis to file.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"text": str, "path": str},
        "output": {"path": str},
        "dependencies": [],
    },
    # --- Touch and State Sensing Capabilities ---
    "c_touch_read": {
        "description": "Read touch sensor state.",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": {"touched": bool},
        "dependencies": [],
    },
    "c_imu_read": {
        "description": "Read IMU data.",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": EOS_TYPE_IMUData,
        "dependencies": [],
    },
    "c_bumper_check": {
        "description": "Check bumper state.",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": {"bumped": bool},
        "dependencies": [],
    },
    "c_temp_read": {
        "description": "Read temperature sensor.",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": {"temperature": float},
        "dependencies": [],
    },
    "c_battery_level": {
        "description": "Read battery level (0-1).",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": {"level": float},
        "dependencies": [],
    },
    "c_clock_time": {
        "description": "Get current timestamp.",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": {"timestamp": float},
        "dependencies": [],
    },
    # --- Lidar Capabilities ---
    "c_lidar_scan": {
        "description": "Get raw lidar scan data.",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": EOS_TYPE_LidarScan,
        "dependencies": [],
    },
    "c_lidar_filter": {
        "description": "Filter lidar scan data.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"scan": EOS_TYPE_LidarScan},
        "output": EOS_TYPE_LidarScan,
        "dependencies": [],
    },
    "c_obstacle_detect": {
        "description": "Detect obstacles from lidar scan.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"scan": EOS_TYPE_LidarScan},
        "output": {"obstacle": bool},
        "dependencies": [],
    },
    "c_ultrasonic_radar_detect": {
        "description": "Detect distance using ultrasonic radar.",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": {"distance": float},
        "dependencies": [],
    },
    # --- Inference Capabilities ---
    "c_model_infer": {
        "description": "General model inference.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"model": str, "tensor": Any},
        "output": {"result": Any},
        "dependencies": [],
    },
    "c_obj_detect": {
        "description": "Object detection from image.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"image": EOS_TYPE_Image},
        "output": {"objects": List[EOS_TYPE_DetectedObject]},
        "dependencies": [],
    },
    "c_pose_estimate": {
        "description": "6D pose estimation from RGB and depth images.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"rgb": EOS_TYPE_Image, "depth": EOS_TYPE_DepthFrame},
        "output": EOS_TYPE_Pose6D,
        "dependencies": [],
    },
    "c_llm_chat": {
        "description": "Natural language chat with LLM.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"prompt": str, "ctx": str},
        "output": {"response": str},
        "dependencies": [],
    },
    "c_segmentation": {
        "description": "Image semantic segmentation.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"image": EOS_TYPE_Image},
        "output": EOS_TYPE_MaskMap,
        "dependencies": [],
    },
    "c_scene_classify": {
        "description": "Scene classification from image.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"image": EOS_TYPE_Image},
        "output": {"label": str},
        "dependencies": [],
    },
    # --- Map and State Capabilities ---
    "c_map_localize": {
        "description": "SLAM localize.",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": EOS_TYPE_Pose2D,
        "dependencies": [],
    },
    "c_tf_lookup": {
        "description": "Transform lookup between frames.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"from": str, "to": str},
        "output": EOS_TYPE_Transform,
        "dependencies": [],
    },
    "c_map_update": {
        "description": "Update map features.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"feature": EOS_TYPE_FeatureList},
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_grid_map_get": {
        "description": "Get current local grid map.",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": EOS_TYPE_GridMap,
        "dependencies": [],
    },
    "c_goal_reached": {
        "description": "Check if goal is reached.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"goal": EOS_TYPE_Pose2D},
        "output": {"reached": bool},
        "dependencies": [],
    },
    "c_insert_semantic_map": {
        "description": "Insert object and position into semantic map.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"name": str, "position": EOS_TYPE_Pose2D},
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_query_semantic_map_by_name": {
        "description": "Query object position by name in semantic map.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"name": str},
        "output": EOS_TYPE_Pose2D,
        "dependencies": [],
    },
    "c_query_semantic_map_by_position": {
        "description": "Query object name by position in semantic map.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"position": EOS_TYPE_Pose2D},
        "output": {"name": str},
        "dependencies": [],
    },
    "c_update_semantic_map": {
        "description": "Update object position in semantic map (overwrite if exists).",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"name": str, "position": EOS_TYPE_Pose2D},
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_delete_semantic_map": {
        "description": "Delete object from semantic map (delete all if name is empty).",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"name": str},
        "output": {"success": bool},
        "dependencies": [],
    },
    # --- System Service Capabilities ---
    "c_file_read": {
        "description": "Read file content as bytes.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"path": str},
        "output": {"data": bytes},
        "dependencies": [],
    },
    "c_file_write": {
        "description": "Write bytes to file.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"path": str, "data": bytes},
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_sql_exec": {
        "description": "Execute SQL query.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"query": str},
        "output": {"rows": List[Dict[str, Any]]},
        "dependencies": [],
    },
    "c_kv_set": {
        "description": "Set key-value pair.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"key": str, "val": Any},
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_kv_get": {
        "description": "Get value by key.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"key": str},
        "output": {"val": Any},
        "dependencies": [],
    },
    "c_json_parse": {
        "description": "Parse JSON string to dict.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"json_str": str},
        "output": {"dict": Dict[str, Any]},
        "dependencies": [],
    },
    "c_md5_hash": {
        "description": "Calculate MD5 hash of data.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"data": bytes},
        "output": {"hash": str},
        "dependencies": [],
    },
    # --- Manipulator and Actuator Capabilities ---
    "c_joint_move": {
        "description": "Move single joint to angle.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"joint_id": str, "angle": float},
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_arm_pose_set": {
        "description": "Set manipulator end-effector to pose.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"pose": EOS_TYPE_Pose6D},
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_gripper_control": {
        "description": "Control gripper gap.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"gap": float},
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_gripper_width_set": {
        "description": "Set gripper width.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"width": float},
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_joint_read": {
        "description": "Read current joint position.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"joint_id": str},
        "output": {"position": float},
        "dependencies": [],
    },
    "c_arm_reset": {
        "description": "Reset manipulator to initial state.",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": {"success": bool},
        "dependencies": [],
    },
    # --- Mobile Chassis Capabilities ---
    "c_chassis_drive": {
        "description": "Set chassis linear and angular velocity.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"v": float, "w": float},
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_chassis_stop": {
        "description": "Stop chassis movement.",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_chassis_local_plan": {
        "description": "Local path planning for chassis.",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"goal": EOS_TYPE_Pose2D},
        "output": EOS_TYPE_Path,
        "dependencies": [],
    },
    "c_chassis_get_pose": {
        "description": "Get current chassis pose.",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": EOS_TYPE_Pose2D,
        "dependencies": [],
    },
    "c_chassis_relocalize": {
        "description": "Relocalize chassis.",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": {"success": bool},
        "dependencies": [],
    },
    # --- Skills ---
    "s_move_arm_to_pose": {
        "description": "Move the manipulator to the specified 6D pose.",
        "type": EOS_SkillType.SKILL,
        "input": {"pose": EOS_TYPE_Pose6D},
        "output": {"success": bool},
        "dependencies": ["c_arm_pose_set"],
    },
    "s_gripper_open": {
        "description": "Open the gripper.",
        "type": EOS_SkillType.SKILL,
        "input": None,
        "output": {"success": bool},
        "dependencies": ["c_gripper_control"],
    },
    "s_gripper_close": {
        "description": "Close the gripper.",
        "type": EOS_SkillType.SKILL,
        "input": None,
        "output": {"success": bool},
        "dependencies": ["c_gripper_control"],
    },
    "s_navigate_to_goal": {
        "description": "Navigate to a target pose with the chassis.",
        "type": EOS_SkillType.SKILL,
        "input": {"goal": EOS_TYPE_Pose2D},
        "output": {"success": bool},
        "dependencies": [
            "c_chassis_local_plan",
            "c_chassis_drive",
            "c_chassis_get_pose",
            "c_goal_reached",
        ],
    },
    "s_grasp_object": {
        "description": "Perceive and grasp the target object.",
        "type": EOS_SkillType.SKILL,
        "input": {"target_name": str},
        "output": {"result": str},
        "dependencies": [
            "s_detect_object",
            "s_estimate_pose",
            "s_move_arm_to_pose",
            "s_gripper_close",
        ],
    },
    "s_pick_and_place": {
        "description": "Pick up an object and place it at the specified pose.",
        "type": EOS_SkillType.SKILL,
        "input": {"target_name": str, "place_pose": EOS_TYPE_Pose6D},
        "output": {"result": str},
        "dependencies": ["s_grasp_object", "s_move_arm_to_pose", "s_gripper_open"],
    },
    "s_patrol_and_avoid": {
        "description": "Patrol along a given path and avoid obstacles.",
        "type": EOS_SkillType.SKILL,
        "input": {"path": List[EOS_TYPE_Pose2D]},
        "output": {"status": str},
        "dependencies": [
            "c_lidar_scan",
            "c_obstacle_detect",
            "c_chassis_drive",
            "c_chassis_get_pose",
        ],
    },
}
