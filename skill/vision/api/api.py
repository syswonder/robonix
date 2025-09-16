from Robonix.manager.eaios_decorators import eaios
import numpy as np
from ultralytics import YOLOE
import traceback
import os
from .vision import px2xy, remove_mask_outliers, get_mask_center_opencv

@eaios.api
@eaios.caller
def skl_detect_objs(camera_name: str) -> dict:
    """
    Detect all objects in the current view of the specified camera and return their categories and coordinates (in the 'map' frame).
    Args:
        camera_name: Name of the camera (e.g., 'camera')
    Returns:
        Dict: {obj_name: (x, y, z)} - Mapping from object name to 3D coordinates in the 'map' frame
    """
    try:
        # Get RGB and depth images
        rgb_image, depth_image = cap_camera_dep_rgb(camera_name)
        if rgb_image is None or depth_image is None:
            print("Failed to get RGB and depth images")
            return {}
        # Get camera parameters
        camera_info = cap_camera_info(camera_name)
        if camera_info is None:
            print("Failed to get camera info")
            return {}
        np_color_image = np.array(rgb_image, dtype=np.uint8)
        np_depth_image = np.array(depth_image, dtype=np.uint16)
        if np_color_image is None:
            return {}
        # Load YOLO model
        # Get the directory where this script is located
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Construct path to model file relative to script location
        model_path = os.path.join(script_dir, "..", "models", "yoloe-11l-seg-pf.pt")
        yolo = YOLOE(model_path)
        # Run YOLO detection
        results = yolo(source=np_color_image, device="cuda:0")
        detection = results[0]
        if detection is None:
            return {}
        # Extract detection results
        object_boxes = detection.boxes.xyxy.cpu().numpy()
        n_objects = object_boxes.shape[0]
        masks = detection.masks.cpu()
        detection_class = detection.boxes.cls.cpu().numpy()
        detection_conf = detection.boxes.conf.cpu().numpy()
        detected_objects = {}
        for i in range(n_objects):
            name = detection.names[detection_class[i]]
            conf = detection_conf[i]
            if conf < 0.7:
                continue
            mask_points = masks.xy[i].reshape(-1, 1, 2).astype(np.int32)
            center_x, center_y = get_mask_center_opencv(mask_points)
            single_selection_mask = np.array(masks.xy[i])
            # Compute object depth
            depths = []
            for point in single_selection_mask:
                p_x = int(point[0])
                p_y = int(point[1])
                if 0 <= p_x < np_depth_image.shape[1] and 0 <= p_y < np_depth_image.shape[0]:
                    depth = np_depth_image.item(p_y, p_x)
                    if not (depth == 0 or depth != depth):
                        depths.append(depth)
            selected_depth = remove_mask_outliers(depths, lower_percentile=10, upper_percentile=70)
            if len(selected_depth) > 0:
                avg_depth = sum(selected_depth) / len(selected_depth)
            else:
                avg_depth = 0
            avg_depth = avg_depth / 1000  # Convert to meters
            center_depth = np_depth_image[int(center_y), int(center_x)] / 1000
            if center_depth == 0 or center_depth != center_depth:
                center_depth = avg_depth
            # Convert pixel to camera coordinates
            world_x, world_y = px2xy([center_x, center_y], camera_info["k"], camera_info["d"], center_depth)
            # Use cap_tf_transform to convert to 'map' frame
            map_x, map_y, map_z = cap_tf_transform('camera_link', 'camera_link', world_x, world_y, center_depth)
            detected_objects[name] = (map_x, map_y, map_z)
        return detected_objects
    except Exception as e:
        print(f"Error in skl_detect_objs: {str(e)}")
        print(f"Traceback: {traceback.format_exc()}")
        return {}
