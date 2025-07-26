import json
import os
import rclpy
from std_srvs.srv import Empty

def load_semantic_map(file_path):
    """
    加载语义地图JSON文件。

    Args:
        file_path (str): 语义地图JSON文件的路径。

    Returns:
        dict: 加载的语义地图数据，如果文件不存在或无效则返回空字典。
    """
    if not os.path.exists(file_path):
        print(f"Error: Semantic map file '{file_path}' not found.")
        return {}
    if os.path.getsize(file_path) == 0:
        print(f"Error: Semantic map file '{file_path}' is empty.")
        return {}
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except json.JSONDecodeError:
        print(f"Error: '{file_path}' is not a valid JSON file.")
        return {}
    except Exception as e:
        print(f"An unexpected error occurred while loading '{file_path}': {e}")
        return {}
    
node = None
lang_map = {
        "人": "person",
        "人机": "person",
        "椅子": "chair",
        "桌子": "table",
        "门": "door",
        "汽车": "car",
        "树": "tree",
}
map_path = "/home/lgw/study/ros_all/EmbodiedAIOS/map/map.json"

def map_query_object(object: str):
    semantic_map_data = load_semantic_map(map_path)
    if object in semantic_map_data:
        return semantic_map_data[object]
    for chinese_object, english_object in lang_map.items():
        if chinese_object == object:
            return semantic_map_data[english_object]
    print(f"Error: '{object}' not found in the semantic map.")
    return None

def map_update():
    rclpy.init()
    if node is None:
        node = rclpy.create_node('piper_vision_api')
    update_map_client = node.create_client(Empty, "/piper_vision/map_capture")
    while not update_map_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('/piper_vision/map_capture service not available, waiting again...')
    request = Empty.Request()
    future = update_map_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info(f"Map update successfully")
    node.destroy_node()
    node = None
    rclpy.shutdown()