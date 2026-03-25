from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class RegisterNodeRequest(_message.Message):
    __slots__ = ("node_id", "namespace", "kind", "skill_md")
    NODE_ID_FIELD_NUMBER: _ClassVar[int]
    NAMESPACE_FIELD_NUMBER: _ClassVar[int]
    KIND_FIELD_NUMBER: _ClassVar[int]
    SKILL_MD_FIELD_NUMBER: _ClassVar[int]
    node_id: str
    namespace: str
    kind: str
    skill_md: str
    def __init__(self, node_id: _Optional[str] = ..., namespace: _Optional[str] = ..., kind: _Optional[str] = ..., skill_md: _Optional[str] = ...) -> None: ...

class RegisterNodeResponse(_message.Message):
    __slots__ = ("node_id",)
    NODE_ID_FIELD_NUMBER: _ClassVar[int]
    node_id: str
    def __init__(self, node_id: _Optional[str] = ...) -> None: ...

class DeclareInterfaceRequest(_message.Message):
    __slots__ = ("node_id", "name", "supported_transports", "metadata_json", "listen_port", "abstract_interface_id")
    NODE_ID_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    SUPPORTED_TRANSPORTS_FIELD_NUMBER: _ClassVar[int]
    METADATA_JSON_FIELD_NUMBER: _ClassVar[int]
    LISTEN_PORT_FIELD_NUMBER: _ClassVar[int]
    ABSTRACT_INTERFACE_ID_FIELD_NUMBER: _ClassVar[int]
    node_id: str
    name: str
    supported_transports: _containers.RepeatedScalarFieldContainer[str]
    metadata_json: str
    listen_port: int
    abstract_interface_id: str
    def __init__(self, node_id: _Optional[str] = ..., name: _Optional[str] = ..., supported_transports: _Optional[_Iterable[str]] = ..., metadata_json: _Optional[str] = ..., listen_port: _Optional[int] = ..., abstract_interface_id: _Optional[str] = ...) -> None: ...

class DeclareInterfaceResponse(_message.Message):
    __slots__ = ("ok", "allocated_endpoint")
    OK_FIELD_NUMBER: _ClassVar[int]
    ALLOCATED_ENDPOINT_FIELD_NUMBER: _ClassVar[int]
    ok: bool
    allocated_endpoint: str
    def __init__(self, ok: bool = ..., allocated_endpoint: _Optional[str] = ...) -> None: ...

class QueryNodesRequest(_message.Message):
    __slots__ = ("namespace", "name", "transport")
    NAMESPACE_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    TRANSPORT_FIELD_NUMBER: _ClassVar[int]
    namespace: str
    name: str
    transport: str
    def __init__(self, namespace: _Optional[str] = ..., name: _Optional[str] = ..., transport: _Optional[str] = ...) -> None: ...

class NodeInfo(_message.Message):
    __slots__ = ("node_id", "namespace", "kind", "interfaces", "has_skill_md")
    NODE_ID_FIELD_NUMBER: _ClassVar[int]
    NAMESPACE_FIELD_NUMBER: _ClassVar[int]
    KIND_FIELD_NUMBER: _ClassVar[int]
    INTERFACES_FIELD_NUMBER: _ClassVar[int]
    HAS_SKILL_MD_FIELD_NUMBER: _ClassVar[int]
    node_id: str
    namespace: str
    kind: str
    interfaces: _containers.RepeatedCompositeFieldContainer[InterfaceInfo]
    has_skill_md: bool
    def __init__(self, node_id: _Optional[str] = ..., namespace: _Optional[str] = ..., kind: _Optional[str] = ..., interfaces: _Optional[_Iterable[_Union[InterfaceInfo, _Mapping]]] = ..., has_skill_md: bool = ...) -> None: ...

class InterfaceInfo(_message.Message):
    __slots__ = ("name", "supported_transports", "metadata_json", "abstract_interface_id")
    NAME_FIELD_NUMBER: _ClassVar[int]
    SUPPORTED_TRANSPORTS_FIELD_NUMBER: _ClassVar[int]
    METADATA_JSON_FIELD_NUMBER: _ClassVar[int]
    ABSTRACT_INTERFACE_ID_FIELD_NUMBER: _ClassVar[int]
    name: str
    supported_transports: _containers.RepeatedScalarFieldContainer[str]
    metadata_json: str
    abstract_interface_id: str
    def __init__(self, name: _Optional[str] = ..., supported_transports: _Optional[_Iterable[str]] = ..., metadata_json: _Optional[str] = ..., abstract_interface_id: _Optional[str] = ...) -> None: ...

class QueryNodesResponse(_message.Message):
    __slots__ = ("nodes",)
    NODES_FIELD_NUMBER: _ClassVar[int]
    nodes: _containers.RepeatedCompositeFieldContainer[NodeInfo]
    def __init__(self, nodes: _Optional[_Iterable[_Union[NodeInfo, _Mapping]]] = ...) -> None: ...

class NegotiateChannelRequest(_message.Message):
    __slots__ = ("consumer_id", "provider_node_id", "interface_name", "transport")
    CONSUMER_ID_FIELD_NUMBER: _ClassVar[int]
    PROVIDER_NODE_ID_FIELD_NUMBER: _ClassVar[int]
    INTERFACE_NAME_FIELD_NUMBER: _ClassVar[int]
    TRANSPORT_FIELD_NUMBER: _ClassVar[int]
    consumer_id: str
    provider_node_id: str
    interface_name: str
    transport: str
    def __init__(self, consumer_id: _Optional[str] = ..., provider_node_id: _Optional[str] = ..., interface_name: _Optional[str] = ..., transport: _Optional[str] = ...) -> None: ...

class NegotiateChannelResponse(_message.Message):
    __slots__ = ("channel_id", "transport", "endpoint")
    CHANNEL_ID_FIELD_NUMBER: _ClassVar[int]
    TRANSPORT_FIELD_NUMBER: _ClassVar[int]
    ENDPOINT_FIELD_NUMBER: _ClassVar[int]
    channel_id: str
    transport: str
    endpoint: str
    def __init__(self, channel_id: _Optional[str] = ..., transport: _Optional[str] = ..., endpoint: _Optional[str] = ...) -> None: ...

class ReleaseChannelRequest(_message.Message):
    __slots__ = ("channel_id",)
    CHANNEL_ID_FIELD_NUMBER: _ClassVar[int]
    channel_id: str
    def __init__(self, channel_id: _Optional[str] = ...) -> None: ...

class ReleaseChannelResponse(_message.Message):
    __slots__ = ("ok",)
    OK_FIELD_NUMBER: _ClassVar[int]
    ok: bool
    def __init__(self, ok: bool = ...) -> None: ...

class QuerySkillMdRequest(_message.Message):
    __slots__ = ("node_id",)
    NODE_ID_FIELD_NUMBER: _ClassVar[int]
    node_id: str
    def __init__(self, node_id: _Optional[str] = ...) -> None: ...

class QuerySkillMdResponse(_message.Message):
    __slots__ = ("skill_md",)
    SKILL_MD_FIELD_NUMBER: _ClassVar[int]
    skill_md: str
    def __init__(self, skill_md: _Optional[str] = ...) -> None: ...

class QueryAllSkillsRequest(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SkillEntry(_message.Message):
    __slots__ = ("node_id", "namespace", "kind", "skill_md")
    NODE_ID_FIELD_NUMBER: _ClassVar[int]
    NAMESPACE_FIELD_NUMBER: _ClassVar[int]
    KIND_FIELD_NUMBER: _ClassVar[int]
    SKILL_MD_FIELD_NUMBER: _ClassVar[int]
    node_id: str
    namespace: str
    kind: str
    skill_md: str
    def __init__(self, node_id: _Optional[str] = ..., namespace: _Optional[str] = ..., kind: _Optional[str] = ..., skill_md: _Optional[str] = ...) -> None: ...

class QueryAllSkillsResponse(_message.Message):
    __slots__ = ("skills",)
    SKILLS_FIELD_NUMBER: _ClassVar[int]
    skills: _containers.RepeatedCompositeFieldContainer[SkillEntry]
    def __init__(self, skills: _Optional[_Iterable[_Union[SkillEntry, _Mapping]]] = ...) -> None: ...

class InspectRuntimeRequest(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class InspectRuntimeResponse(_message.Message):
    __slots__ = ("json",)
    JSON_FIELD_NUMBER: _ClassVar[int]
    json: str
    def __init__(self, json: _Optional[str] = ...) -> None: ...
