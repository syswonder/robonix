# import networkx as nx
# G = nx.Graph(day="Friday")
# print(G.graph)  #输出：{'day': 'Friday'}
# G.add_node(1, time="5pm")
# G.add_nodes_from([3], time="2pm", label="node3")
# print(G.nodes[1])
# print(list(G.nodes(data=True)))

import networkx as nx
import numpy as np
from typing import List, Optional, Dict
from dataclasses import dataclass
import itertools

@dataclass
class MemoryNode:
    # 结构化记忆节点
    node_id: int
    node_type: int  # 0:短期, 1:长期
    node_class: int      # 0:空间, 1:时间-子节点有序, 2:情景
    name: str = "" # 节点名称
    summary: str = ""   # 节点及其下一级子节点摘要-GPT总结/ A 有 a1 a2 a3
    parent_id: int = -1  # 父节点ID
    timestamp: float = "" # 时间戳-短期记忆用于遗忘
    weight: float = 1.0 # 节点权重 小于 0为错误节点？
    has_child: bool = False # 是否有子节点
    # child_ids: List[int] = None  # 子节点ID列表 - 以图为准
    parents_cnt: int = 0  # 父节点数量
    # embedding: np.ndarray  # 单独找个图存储

class MemoryGraph:
    # 带语义检索的混合记忆图谱
    def __init__(self, map_type: int = 0, id_start: int = 0, max_id: int = 1000):
        self.G = nx.DiGraph()
        # self.map_type = map_type # 0:短期, 1:长期
        self._id_counters = itertools.count(id_start)  # 短期ID从0开始 长期ID从1000开始
        self.max_id = max_id  # 最大ID限制
        # 辅助索引结构
        self._node_map = {} # 节点映射

    def forget_node(self)-> bool:
        # 短期记忆遗忘逻辑
        # 实现基于时间戳的遗忘策略
        if self.map_type == 0:
            # 短期记忆
            current_time = np.datetime64('now')
            for node_id, node in list(self._node_map[0].items()):
                if (current_time - node.timestamp) > np.timedelta64(1, 'h'):
                    self.delete_node(node_id)
            return True
        else:
            # 长期记忆不进行遗忘
            print("长期记忆不进行遗忘处理")
            return True

    def _get_next_id(self) -> int:
        # 获取下一个可用ID
        next_id = next(self._id_counters)
        if next_id >= self.max_id:
            # 短期记忆进行遗忘溢出处理
            self.forget_node()
            next_id = next(self._id_counters)
        
        return next_id

    def add_node(self, node_type: int, node_class: int,
                    name: str, summary: str,
                    parent_id: int = -1,
                    timestamp: float = -1.0, weight: float = 1.0,
                    has_child: bool = False, ordered: bool = False) -> int:
        # 添加节点到图
        if node_type not in [0, 1]:
            raise ValueError("node_type must be 0 (short-term) or 1 (long-term)")
        if node_class not in [0, 1, 2]:
            raise ValueError("node_class must be 0 (spatial), 1 (temporal-ordered), or 2 (contextual)")
        node_id = self._get_next_id()
        node = MemoryNode(
            node_id=node_id,
            node_type=node_type,
            node_class=node_class,
            name=name,
            summary=summary,
            parent_id=parent_id,
            timestamp=timestamp,
            weight=weight,
            has_child=has_child,
            parents_cnt=1
        )

        # 添加到图中
        self.G.add_node(node_id, **node.__dict__)
        # 更新辅助索引
        self._node_map[node_id] = node
        # 更新父节点子节点
        if parent_id != -1:
            self.add_child(parent_id, node_id)
        return node_id

    def delete_node(self, node_id: int):
        # 删除节点及其关联关系
        if node_id in self._node_map:
            node = self._node_map[node_id]
            # 删除子节点关系
            if node.has_child:
                # 以图中子节点的为准
                for child_id in list(self.G.successors(node_id)):
                    self._node_map[child_id].parents_cnt -= 1
                    if self._node_map[child_id].parents_cnt == 0:
                        self.delete_node(child_id) # 递归删除无效子节点
            # 删除图中的节点
            self.G.remove_node(node_id)
            # 更新辅助索引 
            del self._node_map[node_id]
    
    def delete_child(self, parent_id: int, child_id: int):
        # 删除边关系
        if parent_id in self._node_map and child_id in self._node_map:
            if self.G.has_edge(parent_id, child_id):
                self.G.remochild_ide_edge(parent_id, child_id)
                # 更新子节点关系
                parent = self._node_map[parent_id]
                child = self._node_map[child_id]
                if child.node_id in parent.child_ids:
                    child.parents_cnt -= 1
                    parent.child_ids.remochild_ide(child.node_id)
                    if parent.parents_cnt == 0:
                        parent.has_child = False
                    # 如果子节点没有父节点了，删除子节点
                    if child.parents_cnt == 0:
                        self.delete_node(child.node_id)


    def add_child(self, parent_id: int, child_id: int):
        # 添加子节点
        if parent_id not in self._node_map:
            raise ValueError(f"Parent node {parent_id} does not exist")
        if child_id not in self._node_map:
            raise ValueError(f"Child node {child_id} does not exist")
        # 获取父节点和子节点对象
        parent_node = self._node_map[parent_id]
        child_node = self._node_map[child_id]
        parent_node.has_child = True
        # 更新child节点的父节点数量
        child_node.parents_cnt += 1
        # 更新图中的边关系
        self.G.add_edge(parent_id, child_node.node_id)




# TODO

    def update_node(self, node_id: int, **kwargs):
        # 更新节点属性

        if node_id in self._node_map[nt]:
            node = self._node_map[nt][node_id]
            for k, v in kwargs.items():
                if hasattr(node, k):
                    setattr(node, k, v)
            # 更新图属性
            attrs = {k: v for k, v in node.__dict__.items() 
                    if k not in ['children', 'node_id']}
            self.G.nodes[node_id].update(attrs)


    def get_node(self, node_id: int) -> Optional[MemoryNode]:
        # 查询节点
        if node_id in self._node_map:
            return self._node_map[node_id]
        return None

    def find_nodes(self, **filters) -> Dict[int, MemoryNode]:
        # 多条件查询节点
        results = {}
        for nid in self._node_map:
            node = self._node_map[nid]
            if all(getattr(node, k, None) == v for k, v in filters.items()):
                results[nid] = node
        return results

# 使用示例
if __name__ == "__main__":
    mg = MemoryGraph()
    
    # 添加短期节点
    # node1 = mg.add_node("Current Location", node2=0, weight=0.8)
    # node2 = mg.add_node("Target Object", node2=1, has_child=True, ordered=True)
    node1_id = mg.add_node(0, 0, "Current Location", "Current location of the agent")
    node2_id = mg.add_node(0, 1, "Target Object", "Object to interact with", parent_id=node1_id, has_child=True, ordered=True)
    
    # 添加长期节点
    node3_id = mg.add_node(1, 0, "Storage Area", "Area where objects are stored")
    node4_id = mg.add_node(1, 2, "Event Log", "Log of past events", parent_id=node3_id)
    
    # 建立关联
    mg.add_child(node1_id, node2_id)
    mg.add_child(node3_id, node4_id)
    mg.add_child(node2_id, node4_id)


    # 查询示例
    print("All nodes in the graph:")
    for node_id, node in mg._node_map.items():
        print(f"Node ID: {node_id}, Name: {node.name}, Summary: {node.summary}, Type: {node.node_type}, Class: {node.node_class}")
    print("\nNode 1 details:", mg.get_node(node1_id).__dict__)
    print("Node 2 details:", mg.get_node(node2_id).__dict__)


    # 更新节点


