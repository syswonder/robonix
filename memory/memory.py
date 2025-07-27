import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import networkx as nx
import numpy as np
from typing import List, Optional, Dict
from dataclasses import dataclass
import itertools
from enum import IntEnum

class NodeType(IntEnum):
    SHORT_TERM = 0   # short-term memory
    LONG_TERM  = 1   # long-term memory
    FIXED      = 2   # pinned memory, not forgotten

class NodeClass(IntEnum):
    SPACE   = 0      # space
    TIME    = 1      # time - ordered sub-nodes
    CONTEXT = 2      # contextual

@dataclass
class MemoryNode:
    # structure for memory nodes
    node_id: int
    node_type: NodeType
    node_class: NodeClass
    name: str = "" # Node name
    summary: str = "" # Summary of the node and its immediate child nodes - GPT summary / A has a1 a2 a3
    parent_id: int = -1 # Parent node ID
    weight: float = 1.0 # Node weight. Less than 0 indicates an error node?
    has_child: bool = False # Has child nodes
    parents_cnt: int = 0 # Number of parent nodes

    # embedding: np.ndarray # Stored in a separate graph
    # optional attributes for specific node types
    # timestamp: float = "" # timestamp for time nodes
    # x: float = 0.0        # optional - spatial node coordinates
    # y: float = 0.0
    # z: float = 0.0

class MemoryGraph:
    def __init__(self, map_type: int = 0, id_start: int = 0, max_id: int = 1000):
        self.G = nx.DiGraph()
        self._id_counters = itertools.count(id_start)  # short start from id_start ; long start from 1000
        self.max_id = max_id  # max ID
        self.time_threshold = 3600  # short-term memory forgetting threshold in seconds
        self._node_map = {} # auxiliary index to store nodes by ID 
    def forget_node(self)-> bool:
        # Short-term memory forgetting logic
        # Implement timestamp-based forgetting strategy
        if self.map_type == 0:
            # Short-term memory
            current_time = np.datetime64('now')
            for node_id, node in list(self._node_map[0].items()):
                if (current_time - node.timestamp) > self.time_threshold:
                    self.delete_node(node_id)
            return True
        else:
            # Long-term memory does not perform forgetting
            print("Long-term memory does not perform forgetting processing")
            return True

    def _get_next_id(self) -> int:
        # Get the next available ID
        next_id = next(self._id_counters)
        if next_id >= self.max_id:
            # Short-term memory performs overflow forgetting processing
            self.forget_node()
            next_id = next(self._id_counters)
        
        return next_id

    def add_node(self, node_type: NodeType, node_class: NodeClass,
                    name: str, summary: str,
                    parent_id: int = -1,
                    timestamp: float = -1.0, weight: float = 1.0,
                    has_child: bool = False, ordered: bool = False,
                    x: float = 0.0, y: float = 0.0, z: float = 0.0) -> int:
        # Add a node to the graph
        if node_type not in [0, 1, 2]:
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
            weight=weight,
            has_child=has_child,
            parents_cnt=1
        )

        # Add to the graph
        self.G.add_node(node_id, **node.__dict__)
        if node_class == NodeClass.TIME and node_type == NodeType.SHORT_TERM :
            # If it is a short-term time node, set it as ordered
            self.G.nodes[node_id]['timestamp'] = timestamp

        if node_class == NodeClass.SPACE:
            # If it is a spatial node, set coordinates
            self.G.nodes[node_id]['x'] = x
            self.G.nodes[node_id]['y'] = y
            self.G.nodes[node_id]['z'] = z

        # Update auxiliary indexes
        self._node_map[node_id] = node
        # Update parent-child relationships
        if parent_id != -1:
            self.add_child(parent_id, node_id)
        return node_id

    def delete_node(self, node_id: int):
        # Delete the node and its associated relationships
        if node_id in self._node_map:
            node = self._node_map[node_id]
            # Delete child node relationships
            if node.has_child:
                # Based on child nodes in the graph
                for child_id in list(self.G.successors(node_id)):
                    self._node_map[child_id].parents_cnt -= 1
                    if self._node_map[child_id].parents_cnt == 0:
                        self.delete_node(child_id) # Recursively delete invalid child nodes
            # Remove the node from the graph
            self.G.remove_node(node_id)
            # Update auxiliary indexes 
            del self._node_map[node_id]

    def delete_child(self, parent_id: int, child_id: int):
        # Delete edge relationships
        if parent_id in self._node_map and child_id in self._node_map:
            if self.G.has_edge(parent_id, child_id):
                self.G.remove_edge(parent_id, child_id)  # Corrected typo: remochild_ide_edge -> remove_edge
                # Update child node relationships
                parent = self._node_map[parent_id]
                child = self._node_map[child_id]
                if child.node_id in parent.child_ids:
                    child.parents_cnt -= 1
                    parent.child_ids.remove(child.node_id)  # Corrected typo: remochild_ide -> remove
                    if parent.parents_cnt == 0:
                        parent.has_child = False
                    # If the child node has no parent nodes left, delete the child node
                    if child.parents_cnt == 0:
                        self.delete_node(child.node_id)

    def add_child(self, parent_id: int, child_id: int):
        # Add a child node
        if parent_id not in self._node_map:
            raise ValueError(f"Parent node {parent_id} does not exist")
        if child_id not in self._node_map:
            raise ValueError(f"Child node {child_id} does not exist")
        # Get parent and child node objects
        parent_node = self._node_map[parent_id]
        child_node = self._node_map[child_id]
        parent_node.has_child = True
        # Update the parent count of the child node
        child_node.parents_cnt += 1
        # Update the edge relationship in the graph
        self.G.add_edge(parent_id, child_node.node_id)

    # TODO: Update basic information + graph information
    def update_node(self, node_id: int, **kwargs):
        # Update node attributes

        if node_id in self._node_map:  # Corrected typo: [nt] -> [] (assuming nt was a typo)
            node = self._node_map[node_id]
            for k, v in kwargs.items():
                if hasattr(node, k):
                    setattr(node, k, v)
            # Update graph attributes
            attrs = {k: v for k, v in node.__dict__.items() 
                    if k not in ['children', 'node_id']}  # Corrected typo: ['children'] added (assuming original intent)
            self.G.nodes[node_id].update(attrs)

    # Basic information
    def get_node(self, node_id: int) -> Optional[MemoryNode]:
        # Query node
        if node_id in self._node_map:
            return self._node_map[node_id]
        return None

    # Get information from the graph
    def get_graph_node(self, node_id: int) -> Dict:
        # Get graph node information
        if node_id in self.G.nodes:
            return self.G.nodes[node_id]
        return {}

    def find_nodes(self, **filters) -> Dict[int, MemoryNode]:
        # Query nodes with multiple conditions
        results = {}
        for nid in self._node_map:
            node = self._node_map[nid]
            if all(getattr(node, k, None) == v for k, v in filters.items()):
                results[nid] = node
        return results

    def load_from_file(self, file_path: str):
        # Load graph data from a file
        # TODO: Implement logic to load graph data from a file
        pass

    def save_to_file(self, file_path: str):
        # Save graph data to a file
        # TODO: Implement logic to save graph data to a file
        pass

    # Convert the graph to a visualization image and save it
    def visualize(self, file_path: str):

        # font_path = '/usr/share/fonts/opentype/noto//NotoSansCJK-Regular.ttc'   # Modify as needed
        # my_font = fm.FontProperties(fname=font_path, size=10)

        # Visualize the graph and save it as an image, using the 'name' field as the node label
        pos = nx.spring_layout(self.G)
        plt.figure(figsize=(12, 8))
        nx.draw(self.G, pos,
                with_labels=False, node_size=700,
                node_color='lightblue',
                font_size=10, font_color='black', font_weight='bold', font_family=my_font.get_name(),
                arrows=True)
        # labels = {node: f"{data['name']}\n{data['summary']}" for node, data in self.G.nodes(data=True)}
        labels = {node: f"{data['name']}" for node, data in self.G.nodes(data=True)}
        nx.draw_networkx_labels(self.G, pos, labels=labels, font_size=8, font_color='black')
        plt.title("Memory Graph Visualization")

        plt.savefig(file_path)
        plt.close()

if __name__ == "__main__":
    mg = MemoryGraph()
    
    node1_id = mg.add_node(NodeType.FIXED, NodeClass.CONTEXT, "embody ai assiant", "as an embody ai assisant")
    node2_id = mg.add_node(NodeType.LONG_TERM, NodeClass.CONTEXT, "skill:get milk", "倒牛奶流程有", parent_id=node1_id, has_child=True, ordered=True)
    node3_id = mg.add_node(NodeType.LONG_TERM, NodeClass.SPACE, "location:kitchen", "厨房里有冰箱，水杯", parent_id=node1_id, x= 1.0, y=1.0, z=1.0)
    node4_id = mg.add_node(NodeType.LONG_TERM, NodeClass.TIME, "open fridge", "打开冰箱", parent_id=node2_id, has_child=True, ordered=True)
    node5_id = mg.add_node(NodeType.LONG_TERM, NodeClass.TIME, "get milk", "取出牛奶", parent_id=node2_id, has_child=True, ordered=True)
    node6_id = mg.add_node(NodeType.LONG_TERM, NodeClass.TIME, "drop bottle", "倒入杯子", parent_id=node2_id, has_child=True, ordered=True)
    node7_id = mg.add_node(NodeType.LONG_TERM, NodeClass.TIME, "heat milk", "加热牛奶", parent_id=node2_id, has_child=True, ordered=True)
    node8_id = mg.add_node(NodeType.LONG_TERM, NodeClass.SPACE, "fridge", "冰箱里有牛奶", parent_id=node3_id, x=1.0, y=2.0, z=1.0)
    node9_id = mg.add_node(NodeType.LONG_TERM, NodeClass.SPACE, "bottle", "水杯在桌子上", parent_id=node3_id, x=2.0, y=3.0, z=1.0)
    node10_id = mg.add_node(NodeType.LONG_TERM, NodeClass.SPACE, "milk", "牛奶在冰箱里", parent_id=node8_id, x=1.0, y=2.0, z=3.0)

    # build temporal and spatial relationships
    mg.add_child(node4_id, node8_id)
    mg.add_child(node5_id, node10_id)
    mg.add_child(node6_id, node9_id)
    mg.add_child(node7_id, node9_id)

    # check
    print("All nodes in the graph:")
    for node_id, node in mg._node_map.items():
        print(f"Node ID: {node_id}, Name: {node.name}, Summary: {node.summary}, Type: {node.node_type}, Class: {node.node_class}")
    print("\nNode 1 details:", mg.get_node(node1_id).__dict__)
    print("Node 2 details:", mg.get_node(node2_id).__dict__)
    print("Node 3 details:", mg.get_node(node3_id).__dict__)
    print("Graph Node 3 details:", mg.get_graph_node(node3_id))

    # visualize
    mg.visualize("memory_graph.png")

