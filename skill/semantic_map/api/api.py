from typing import Dict, Tuple, Optional
from DeepEmbody.manager.eaios_decorators import eaios
from .map import semantic_map

@eaios.api
@eaios.caller
def skl_update_map(camera_name: str) -> bool:
    """
    当前视角下,识别所有物体,加入语义地图
    (Under the current view, identify all objects and add them to the semantic map)
    
    Returns:
        bool: 是否成功 (whether successful)
    """
    try:
        # Use skl_detect_objs to get objects and their coordinates from camera
        detected_objects = skl_detect_objs(camera_name)
        
        if not detected_objects:
            print("No objects detected in current view")
            return False
        
        # Add all detected objects to the semantic map
        success_count = 0
        for obj_name, coordinates in detected_objects.items():
            if semantic_map.add_object(obj_name, coordinates):
                success_count += 1
                print(f"Added object '{obj_name}' at coordinates {coordinates}")
            else:
                print(f"Failed to add object '{obj_name}' to semantic map")
        
        print(f"Successfully added {success_count}/{len(detected_objects)} objects to semantic map")
        return success_count > 0
        
    except Exception as e:
        print(f"Error in skl_update_map: {e}")
        return False

@eaios.api
@eaios.caller
def skl_query_map_all() -> Dict[str, Tuple[float, float, float]]:
    """
    查询语义地图所有物体
    (Query all objects in the semantic map)
    
    Returns:
        Dict: {obj_name: str, (x, y, z)} - Dictionary mapping object names to coordinates
    """
    try:
        all_objects = semantic_map.get_all_objects()
        print(f"Found {len(all_objects)} objects in semantic map")
        return all_objects
    except Exception as e:
        print(f"Error in skl_query_map_all: {e}")
        return {}

@eaios.api
@eaios.caller
def skl_query_map(obj_name: str) -> Optional[Tuple[float, float, float]]:
    """
    查询语义地图指定物体
    (Query a specified object in the semantic map)
    
    Args:
        obj_name: str - Name of the object to query
        
    Returns:
        (x, y, z): (float, float, float) - Coordinates of the specified object, or None if not found
    """
    try:
        coordinates = semantic_map.get_object(obj_name)
        if coordinates:
            print(f"Found object '{obj_name}' at coordinates {coordinates}")
        else:
            print(f"Object '{obj_name}' not found in semantic map")
        return coordinates
    except Exception as e:
        print(f"Error in skl_query_map: {e}")
        return None

@eaios.api
@eaios.caller
def skl_add_map_obj(obj_name: str, coordinates: Tuple[float, float, float]) -> bool:
    """
    手动向语义地图中增加物体
    (Manually add an object to the semantic map)
    
    Args:
        obj_name: str - Name of the object
        coordinates: (float, float, float) - (x, y, z) coordinates of the object
        
    Returns:
        bool: 是否成功 (whether successful)
    """
    try:
        success = semantic_map.add_object(obj_name, coordinates)
        if success:
            print(f"Successfully added object '{obj_name}' at coordinates {coordinates}")
        else:
            print(f"Failed to add object '{obj_name}' to semantic map")
        return success
    except Exception as e:
        print(f"Error in skl_add_map_obj: {e}")
        return False
