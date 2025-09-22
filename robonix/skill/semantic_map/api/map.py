import json
import os
from typing import Dict, Tuple, Optional
import threading

class SemanticMap:
    """
    Semantic map storage class for managing objects and their coordinates
    """
    
    def __init__(self, map_file_path: str = None):
        if map_file_path is None:
            # 获取当前文件所在目录的路径
            current_dir = os.path.dirname(os.path.abspath(__file__))
            # 构建map目录的路径
            map_dir = os.path.join(os.path.dirname(current_dir), "map")
            # 确保map目录存在
            os.makedirs(map_dir, exist_ok=True)
            # 设置默认的map文件路径
            self.map_file_path = os.path.join(map_dir, "semantic_map.json")
        else:
            self.map_file_path = map_file_path
        self._lock = threading.Lock()
        self._objects: Dict[str, Tuple[float, float, float]] = {}
        self._load_map()
    
    def _load_map(self):
        """Load semantic map from file"""
        try:
            if os.path.exists(self.map_file_path):
                with open(self.map_file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    # Convert list format to tuple format for coordinates
                    for obj_name, coords in data.items():
                        if isinstance(coords, list) and len(coords) == 3:
                            self._objects[obj_name] = tuple(coords)
                        elif isinstance(coords, tuple) and len(coords) == 3:
                            self._objects[obj_name] = coords
        except Exception as e:
            print(f"Warning: Failed to load semantic map from {self.map_file_path}: {e}")
            self._objects = {}
    
    def _save_map(self):
        """Save semantic map to file"""
        try:
            with open(self.map_file_path, 'w', encoding='utf-8') as f:
                json.dump(self._objects, f, indent=4, ensure_ascii=False)
        except Exception as e:
            print(f"Error: Failed to save semantic map to {self.map_file_path}: {e}")
    
    def add_object(self, obj_name: str, coordinates: Tuple[float, float, float]) -> bool:
        """
        Add or update an object in the semantic map
        
        Args:
            obj_name: Name of the object
            coordinates: (x, y, z) coordinates of the object
            
        Returns:
            bool: True if successful, False otherwise
        """
        x, y, z = coordinates
        if x is None or y is None or z is None:
            print(f"Error: Invalid coordinates for object {obj_name}: {coordinates}")
            return False
        try:
            with self._lock:
                self._objects[obj_name] = coordinates
                self._save_map()
                return True
        except Exception as e:
            print(f"Error adding object {obj_name} to semantic map: {e}")
            return False
    
    def get_object(self, obj_name: str) -> Optional[Tuple[float, float, float]]:
        """
        Get coordinates of a specific object
        
        Args:
            obj_name: Name of the object to query
            
        Returns:
            Tuple[float, float, float]: (x, y, z) coordinates of the object, or None if not found
        """
        with self._lock:
            return self._objects.get(obj_name)
    
    def get_all_objects(self) -> Dict[str, Tuple[float, float, float]]:
        """
        Get all objects in the semantic map
        
        Returns:
            Dict[str, Tuple[float, float, float]]: Dictionary mapping object names to coordinates
        """
        with self._lock:
            return self._objects.copy()
    
    def clear_map(self):
        """Clear all objects from the semantic map"""
        with self._lock:
            self._objects.clear()
            self._save_map()

# Global semantic map instance
semantic_map = SemanticMap()
