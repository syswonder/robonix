"""
Types Module
============

This module defines common data types and enumerations used throughout
the robonix OS system. It includes image formats, camera types,
skill types, and various data structures for system operations.
"""

from enum import Enum
from dataclasses import dataclass
from typing import List, Tuple
import datetime


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


class EOS_TYPE_Datetime:
    timestamp: datetime.datetime


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
class EOS_TYPE_Pose2D:
    x: float
    y: float
    yaw: float


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
class EOS_TYPE_Transform:
    translation: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]  # Quaternion


@dataclass
class EOS_TYPE_Path:
    poses: List[EOS_TYPE_Pose2D]


EntityPath = str  # Represents the path of an entity.

EntityPathAndRequired = {
    "entity": EntityPath,
    "required": List[str],
}


# --- SpatialLM Detection Result Types ---

@dataclass
class EOS_TYPE_SpatialLM_Wall:
    """Wall detection result from SpatialLM"""
    id: str
    ax: float  # start point x
    ay: float  # start point y
    az: float  # start point z
    bx: float  # end point x
    by: float  # end point y
    bz: float  # end point z
    height: float
    thickness: float
    world_center_x: float  # geometric center in world coordinates
    world_center_y: float
    world_center_z: float


@dataclass
class EOS_TYPE_SpatialLM_Door:
    """Door detection result from SpatialLM"""
    id: str
    wall_id: str
    position_x: float
    position_y: float
    position_z: float
    width: float
    height: float
    world_center_x: float  # geometric center in world coordinates
    world_center_y: float
    world_center_z: float


@dataclass
class EOS_TYPE_SpatialLM_Window:
    """Window detection result from SpatialLM"""
    id: str
    wall_id: str
    position_x: float
    position_y: float
    position_z: float
    width: float
    height: float
    world_center_x: float  # geometric center in world coordinates
    world_center_y: float
    world_center_z: float


@dataclass
class EOS_TYPE_SpatialLM_Bbox:
    """Bounding box detection result from SpatialLM"""
    id: str
    class_name: str
    position_x: float
    position_y: float
    position_z: float
    angle_z: float
    scale_x: float
    scale_y: float
    scale_z: float
    world_center_x: float  # geometric center in world coordinates
    world_center_y: float
    world_center_z: float


@dataclass
class EOS_TYPE_SpatialLM_WorldResult:
    """Complete SpatialLM detection result with world coordinates"""
    walls: List[EOS_TYPE_SpatialLM_Wall]
    doors: List[EOS_TYPE_SpatialLM_Door]
    windows: List[EOS_TYPE_SpatialLM_Window]
    bboxes: List[EOS_TYPE_SpatialLM_Bbox]
    robot_pose: Tuple[float, float, float, float]  # (x, y, z, yaw) of robot when detection was made
