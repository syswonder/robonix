from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Empty(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class Observation(_message.Message):
    __slots__ = ("rgb", "depth", "width", "height", "proprio", "done", "reward", "fx", "fy", "cx", "cy", "goal_pos", "camera_pose")
    RGB_FIELD_NUMBER: _ClassVar[int]
    DEPTH_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    PROPRIO_FIELD_NUMBER: _ClassVar[int]
    DONE_FIELD_NUMBER: _ClassVar[int]
    REWARD_FIELD_NUMBER: _ClassVar[int]
    FX_FIELD_NUMBER: _ClassVar[int]
    FY_FIELD_NUMBER: _ClassVar[int]
    CX_FIELD_NUMBER: _ClassVar[int]
    CY_FIELD_NUMBER: _ClassVar[int]
    GOAL_POS_FIELD_NUMBER: _ClassVar[int]
    CAMERA_POSE_FIELD_NUMBER: _ClassVar[int]
    rgb: bytes
    depth: bytes
    width: int
    height: int
    proprio: _containers.RepeatedScalarFieldContainer[float]
    done: bool
    reward: float
    fx: float
    fy: float
    cx: float
    cy: float
    goal_pos: _containers.RepeatedScalarFieldContainer[float]
    camera_pose: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, rgb: _Optional[bytes] = ..., depth: _Optional[bytes] = ..., width: _Optional[int] = ..., height: _Optional[int] = ..., proprio: _Optional[_Iterable[float]] = ..., done: bool = ..., reward: _Optional[float] = ..., fx: _Optional[float] = ..., fy: _Optional[float] = ..., cx: _Optional[float] = ..., cy: _Optional[float] = ..., goal_pos: _Optional[_Iterable[float]] = ..., camera_pose: _Optional[_Iterable[float]] = ...) -> None: ...

class Action(_message.Message):
    __slots__ = ("values",)
    VALUES_FIELD_NUMBER: _ClassVar[int]
    values: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, values: _Optional[_Iterable[float]] = ...) -> None: ...

class StepResult(_message.Message):
    __slots__ = ("obs", "reward", "done", "info_json")
    OBS_FIELD_NUMBER: _ClassVar[int]
    REWARD_FIELD_NUMBER: _ClassVar[int]
    DONE_FIELD_NUMBER: _ClassVar[int]
    INFO_JSON_FIELD_NUMBER: _ClassVar[int]
    obs: Observation
    reward: float
    done: bool
    info_json: str
    def __init__(self, obs: _Optional[_Union[Observation, _Mapping]] = ..., reward: _Optional[float] = ..., done: bool = ..., info_json: _Optional[str] = ...) -> None: ...
