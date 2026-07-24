"""
记忆节点数据结构定义
- 枚举类型: NodeType, NodeClass, EdgeType, ModalityType
- 数据类: MemoryNode
"""

from enum import IntEnum, Enum
from dataclasses import dataclass, field
from typing import Dict, Any, Optional
import numpy as np


# ======================== 枚举定义 ========================

class NodeType(IntEnum):
    """节点生命周期类型"""
    SHORT_TERM = 0   # 短期记忆（可被遗忘/可提升）
    LONG_TERM  = 1   # 长期记忆（不被遗忘，检索提升而来）
    FIXED      = 2   # 固定记忆（初始化写入，永不遗忘）
    SKG_EVENT  = 3   # 事理知识图谱事件节点


class NodeClass(IntEnum):
    """节点语义分类"""
    CONTEXT = 0      # 情景/技能节点
    TIME    = 1      # 时间有序节点
    SPACE   = 2      # 空间节点


class EdgeType(str, Enum):
    """边关系类型"""
    PARENT_CHILD = "child"      # 层级从属（父子关系）
    NEXT         = "next"       # SKG 因果/时序后继
    REQUIRES     = "requires"   # 前置依赖
    PRODUCES     = "produces"   # 产出关系


class ModalityType(str, Enum):
    """多模态数据类型标识"""
    TEXT       = "text"        # 纯文本描述
    IMAGE      = "image"       # 图像（RGB截图/VLM检测结果）
    DEPTH      = "depth"       # 深度图
    SPATIAL    = "spatial"     # 3D坐标 (x, y, z)
    INVENTORY  = "inventory"   # 背包状态（结构化dict）
    EVENT      = "event"       # 事件要素 (A/O/T/V/P/L)


# ======================== 数据类 ========================

@dataclass
class MemoryNode:
    """记忆节点，支持多模态数据"""

    # --- 基础属性 ---
    node_id: int = -1
    node_type: NodeType = NodeType.SHORT_TERM
    node_class: NodeClass = NodeClass.CONTEXT
    name: str = ""
    summary: str = ""
    parent_id: int = -1
    weight: float = 1.0
    child_cnt: int = 0
    parents_cnt: int = 0

    # --- 检索/提升相关 ---
    hit_count: int = 0

    # --- 多模态数据容器 ---
    modalities: Dict[str, Any] = field(default_factory=dict)

    # --- 向量嵌入缓存 ---
    embedding: Optional[np.ndarray] = None

    def to_dict(self) -> dict:
        """序列化为字典（不含 embedding）"""
        ...

    @classmethod
    def from_dict(cls, data: dict) -> "MemoryNode":
        """从字典反序列化"""
        ...


# ======================== 测试 ========================

def test():
    """演示 MemoryNode 及枚举的基本用法"""

    # 1. 创建一个短期空间节点
    space_node = MemoryNode(
        node_id=0,
        node_type=NodeType.SHORT_TERM,
        node_class=NodeClass.SPACE,
        name="log_2_1_-2",
        summary="a log block nearby",
        weight=1.0,
    )
    print(f"[创建空间节点] id={space_node.node_id}, name={space_node.name}, "
          f"type={space_node.node_type.name}, class={space_node.node_class.name}")

    # 2. 为空间节点写入多模态数据
    space_node.modalities[ModalityType.SPATIAL] = {
        "x": 2.0, "y": 1.0, "z": -2.0, "yaw": 90.0, "pitch": 0.0
    }
    space_node.modalities[ModalityType.IMAGE] = {
        "path": "log/frame_001.png",
        "vlm_labels": ["log", "grass"],
    }
    print(f"[写入模态] spatial={space_node.modalities[ModalityType.SPATIAL]}")
    print(f"[写入模态] image={space_node.modalities[ModalityType.IMAGE]}")

    # 3. 创建一个 SKG 事件节点
    event_node = MemoryNode(
        node_id=100,
        node_type=NodeType.SKG_EVENT,
        node_class=NodeClass.TIME,
        name="attack_log",
        summary="attack the log block",
    )
    event_node.modalities[ModalityType.EVENT] = {
        "A": "attack", "O": "log", "T": 1,
        "V": "forest", "P": "mine_log", "L": "attack the log"
    }
    event_node.modalities[ModalityType.INVENTORY] = {
        "input_items": {}, "output_items": {"log": 1}
    }
    print(f"\n[创建事件节点] id={event_node.node_id}, name={event_node.name}")
    print(f"[事件要素] {event_node.modalities[ModalityType.EVENT]}")

    # 4. 模拟检索命中 → hit_count 增长 → 提升判断
    promote_threshold = 3
    for i in range(promote_threshold):
        space_node.hit_count += 1
        print(f"[命中] hit_count={space_node.hit_count}")
    if space_node.hit_count >= promote_threshold and space_node.node_type == NodeType.SHORT_TERM:
        space_node.node_type = NodeType.LONG_TERM
        print(f"[提升] {space_node.name}: SHORT_TERM → LONG_TERM")

    # 5. 序列化 / 反序列化
    d = space_node.to_dict()
    print(f"\n[序列化] {d}")
    restored = MemoryNode.from_dict(d)
    print(f"[反序列化] id={restored.node_id}, name={restored.name}, type={restored.node_type.name}")

    # 6. 边类型枚举遍历
    print("\n[边类型枚举]")
    for et in EdgeType:
        print(f"  {et.name} = {et.value}")


if __name__ == "__main__":
    test()
