"""
SKG 中间层 — ScriptKnowledgeGraph
- 5 个公共接口: load / save / retrieve / insert_experience / update_short_term / feedback_failure
- 内部: Retriever(向量检索) + 对齐算法 + 短期→长期自动提升 + 失败降权



"""

from typing import List, Dict, Optional, Any

from memory_node import MemoryNode, NodeType, NodeClass, EdgeType, ModalityType
from memory_graph import MemoryGraph


class ScriptKnowledgeGraph: # ekg

    def __init__(self, memory_graph: MemoryGraph, skg_path: str = None):
        self.mg = memory_graph
        self.retriever = None               # 惰性初始化
        self._skg_path = skg_path or "memory/data/skg.json"
        self._retriever_dirty = True        # 是否需要重建索引
        self._skg_id_map: Dict[str, int] = {}   # skg string ID ↔ mg int ID
        self._promote_threshold = 3         # 短期→长期提升的命中次数阈值
        self._failure_decay = 0.5           # 失败反馈的权重衰减因子

    # ======================== 公共接口 ========================

    def load(self, scene_info: dict, skg_path: str = None):
        """
        初始化/加载记忆
        - 从 scene_info 构建空间/技能节点到 MemoryGraph
        - 从 skg.json 读取事件，创建 SKG_EVENT 节点并用 NEXT 边建立因果链
        - 维护 _skg_id_map
        """
        ...

    def save(self):
        """
        持久化
        - 调用 mg.save_to_file() 保存图
        - 调用 _sync_to_skg_json() 导出 SKG_EVENT 为 skg.json 格式
        """
        ...

    def retrieve(self, query: str, top_k: int = 5) -> List[dict]:
        """
        记忆检索
        - 惰性初始化/更新 Retriever
        - 向量检索 + weight 加权排序 (score = cosine_sim * weight)
        - SKG_EVENT 命中时沿 NEXT 边追踪完整事件链
        - 每个命中节点 hit_count++ → 调用 _try_promote() 检查提升
        - 返回: [{"name", "summary", "event", "chain", "chain_node_ids"}, ...]
        """
        ...

    def insert_experience(self, task_describe: str,
                          record_actions: list) -> dict:
        """
        成功经验插入（含对齐）
        - 语义粗筛: Retriever 余弦相似度 ≥ 0.7
        - 结构精匹配: 动作序列 LCS 比率 ≥ 0.6
        - 匹配成功 → _merge_event_chain()
        - 匹配失败 → _create_event_chain()
        - 返回 task_spec: {"name", "description", "input", "output", "actions"}
        """
        ...

    def update_short_term(self, obj_list: list, entity: dict):
        """
        短期空间记忆更新
        - 找到 /temp 空间根节点
        - 增量更新: 比较新旧物体集合，仅增删差异部分
        - 新节点初始化 hit_count=0, weight=1.0
        - 标记 _retriever_dirty = True
        """
        ...

    def feedback_failure(self, chain_node_ids: List[int]):
        """
        失败经验反馈 — 降权事件链
        - 对链中所有节点: weight *= _failure_decay
        - 权重下限 0.1
        - 调用 _sync_to_skg_json() 持久化
        """
        ...

    # ======================== 内部方法 ========================

    def _align_and_merge(self, task_describe: str,
                         action_names: List[str]) -> Optional[int]:
        """
        对齐: 语义粗筛 + 结构精匹配
        返回: 匹配到的已有链根节点ID，无匹配返回 None
        """
        ...

    def _semantic_match(self, query: str, candidates: List[dict],
                        threshold: float = 0.7) -> List[dict]:
        """
        语义粗筛: 用 Retriever 编码 query 与候选比较余弦相似度
        返回: 超过阈值的候选列表（附 score）
        """
        ...

    def _structural_match(self, new_actions: List[str],
                          existing_chain_id: int,
                          threshold: float = 0.6) -> float:
        """
        结构精匹配: 已有链动作序列 vs 新动作序列的 LCS 比率
        返回: 2*len(LCS) / (len(a) + len(b))
        """
        ...

    def _create_event_chain(self, task_describe: str,
                            record_actions: list) -> int:
        """
        创建新事件链
        - 任务根节点 (SKG_EVENT/CONTEXT)
        - 每步动作节点 (SKG_EVENT/TIME) + NEXT 边串联
        - 写入 event/inventory/spatial 模态数据
        返回: 根节点ID
        """
        ...

    def _merge_event_chain(self, chain_id: int, task_describe: str,
                           record_actions: list):
        """
        合并到已有事件链
        - 增加根节点权重
        - 追加新步骤节点（如有额外步骤）
        """
        ...

    def _try_promote(self, node_id: int):
        """
        检查 hit_count >= threshold
        若是且当前为 SHORT_TERM → 提升为 LONG_TERM
        """
        ...

    def _sync_to_skg_json(self):
        """将所有 SKG_EVENT 节点导出为 skg.json 格式"""
        ...

    def _load_skg_json(self, path: str):
        """读取 skg.json，创建 SKG_EVENT 节点 + NEXT 边"""
        ...


# ======================== 测试 ========================

def test():
    """演示 SKG 中间层完整调用流程"""

    # ---- 0. 初始化底层图 + SKG ----
    mg = MemoryGraph(id_start=0, max_id=500)
    skg = ScriptKnowledgeGraph(mg, skg_path="/tmp/test_skg.json")
    print("=== 初始化完成 ===")

    # ---- 1. load: 加载场景 + 已有 skg ----
    scene_info = {
        "/": {"name": "/", "summary": "root"},
        "/temp": {"name": "/temp", "summary": "space root"},
        "/temp/log_2_1_-2": {
            "name": "log_2_1_-2", "parent": "/temp",
            "RElx": 2, "REly": 1, "RElz": -2, "size": 3
        },
        "/temp/grass_2_-1_0": {
            "name": "grass_2_-1_0", "parent": "/temp",
            "RElx": 2, "REly": -1, "RElz": 0, "size": 8
        },
    }
    skg.load(scene_info)
    print("[load] scene loaded into MemoryGraph")

    # ---- 2. update_short_term: 环境变化，增量更新 ----
    new_obj_list = [
        {"name": "log", "RELx": 2, "RELy": 1, "RELz": -2, "size": 3},
        {"name": "dirt", "RELx": 1, "RELy": -2, "RELz": 0, "size": 12},
        # grass 消失了 → 应被删除
    ]
    entity = {"x": 100.5, "y": 64.0, "z": -200.3, "yaw": 90.0, "pitch": 0.0}
    skg.update_short_term(new_obj_list, entity)
    print("[update_short_term] obj_list updated (grass removed, dirt added)")

    # ---- 3. insert_experience: 插入成功经验 ----
    record_actions = [
        {
            "action": "move",
            "input": {"aimed_obj": "log", "obj_list": [{"name": "log"}]},
            "output": {"output_items": {}},
        },
        {
            "action": "attack",
            "input": {"aimed_obj": "log", "obj_list": [{"name": "log"}]},
            "output": {"output_items": {"log": 1}},
        },
        {
            "action": "attack",
            "input": {"aimed_obj": "log", "obj_list": []},
            "output": {"output_items": {"log": 2}},
        },
    ]
    task_spec = skg.insert_experience("mine log from tree", record_actions)
    print(f"[insert_experience] created task: {task_spec}")

    # ---- 4. insert_experience 再次: 相似任务应触发合并 ----
    record_actions_2 = [
        {"action": "move", "input": {"aimed_obj": "log"}, "output": {}},
        {"action": "attack", "input": {"aimed_obj": "log"}, "output": {"output_items": {"log": 1}}},
        {"action": "attack", "input": {"aimed_obj": "log"}, "output": {"output_items": {"log": 2}}},
        {"action": "look", "input": {"aimed_obj": "log"}, "output": {}},  # 新增步骤
    ]
    task_spec_2 = skg.insert_experience("mine log", record_actions_2)
    print(f"[insert_experience] merged/new task: {task_spec_2}")

    # ---- 5. retrieve: 检索记忆 ----
    results = skg.retrieve("how to get log", top_k=3)
    print(f"[retrieve] got {len(results)} results:")
    for r in results:
        print(f"  name={r.get('name')}, chain={r.get('chain')}, "
              f"chain_node_ids={r.get('chain_node_ids')}")

    # ---- 6. 模拟执行失败 → feedback_failure ----
    if results and results[0].get("chain_node_ids"):
        failed_ids = results[0]["chain_node_ids"]
        skg.feedback_failure(failed_ids)
        print(f"[feedback_failure] decayed weights for chain: {failed_ids}")

        # 再次检索，失败链应排名靠后
        results_after = skg.retrieve("how to get log", top_k=3)
        print(f"[retrieve after failure] top result: {results_after[0].get('name') if results_after else 'none'}")

    # ---- 7. 多次检索触发短期→长期提升 ----
    for i in range(skg._promote_threshold):
        skg.retrieve("log block nearby", top_k=1)
    # 检查 log 空间节点是否被提升
    short_nodes = mg.find_nodes(node_type=NodeType.SHORT_TERM, node_class=NodeClass.SPACE)
    long_nodes = mg.find_nodes(node_type=NodeType.LONG_TERM, node_class=NodeClass.SPACE)
    print(f"[auto promote] SHORT_TERM space={len(short_nodes)}, LONG_TERM space={len(long_nodes)}")

    # ---- 8. save: 持久化 ----
    skg.save()
    print("[save] graph + skg.json saved")

    # ---- 9. 重新加载验证 ----
    mg3 = MemoryGraph()
    skg3 = ScriptKnowledgeGraph(mg3, skg_path="/tmp/test_skg.json")
    skg3.load(scene_info)
    results3 = skg3.retrieve("mine log", top_k=1)
    print(f"[reload verify] retrieve after reload: {results3}")

    print("\n=== 全部接口测试完成 ===")


if __name__ == "__main__":
    test()
