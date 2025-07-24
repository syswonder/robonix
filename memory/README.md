# Memory

Stores information across and during tasks. Can include:

- JIT prompts
- World model fragments (object location, room map)
- Short-term memory (task context, intermediate results)
- Long-term skill usage patterns

Memory enables contextual reasoning, personalization, and short-term task coherence.



# 构建语义森林
import networkx as nx  # 图网络操作库

generate_semantic_forest.py
    self.G = nx.Graph()  # 网络图对象

    llm_interface = LLMInterface()  # 语言模型接口  from embodied_nav.llm import LLMInterface  # 大型语言模型接口
    


# 检索与问答
experiment.py


如何构建的-networkx构建，节点有各种属性

该项目中的embedding是什么

如何检索查询的


如何对比的

tar -cvf memory.tar memory

class MemoryNode:
    # 结构化记忆节点
    node_id: int
    node_type: int  # 0:短期, 1:长期
    node_class: int      # 0:空间, 1:时间-子节点有序, 2:情景
    name: str = "" # 节点名称
    summary: str = ""   # 节点及其下一级子节点摘要
    parent_id: int = -1  # 父节点ID - 有多个父节点时，有可能原始父节点无效
    timestamp: float = "" # 时间戳-短期记忆用于遗忘
    weight: float = 1.0 # 节点权重 小于 0为错误节点？
    has_child: bool = False # 是否有子节点
    # child_ids: List[int] = None  # 子节点ID列表 - 以图为准
    parents_cnt: int = 0  # 父节点数量
    # embedding: np.ndarray  # 单独找个图存储

DiGraph（有向图）