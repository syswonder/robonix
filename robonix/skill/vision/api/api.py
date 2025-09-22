import numpy as np
from ultralytics import YOLOE
import traceback
import os
import cv2
import random
from datetime import datetime
from .vision import px2xy, remove_mask_outliers, get_mask_center_opencv

from robonix.manager.eaios_decorators import eaios

def create_color_palette(n_colors):
    colors = []
    for i in range(n_colors):
        hue = (i * 137.508) % 360
        saturation = 70 + random.randint(-20, 20)
        value = 80 + random.randint(-20, 20)

        hsv = np.array([[[hue, saturation, value]]], dtype=np.uint8)
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        colors.append(bgr[0, 0].tolist())
    return colors

def draw_rounded_rectangle(img, x1, y1, x2, y2, color, thickness=2, radius=10):
    # Draw main rectangle
    cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)

    # Draw rounded corners
    cv2.circle(img, (x1 + radius, y1 + radius), radius, color, -1)
    cv2.circle(img, (x2 - radius, y1 + radius), radius, color, -1)
    cv2.circle(img, (x1 + radius, y2 - radius), radius, color, -1)
    cv2.circle(img, (x2 - radius, y2 - radius), radius, color, -1)

def put_text_with_background(
    img,
    text,
    position,
    font_scale=0.6,
    thickness=2,
    bg_color=(0, 0, 0),
    text_color=(255, 255, 255),
):
    font = cv2.FONT_HERSHEY_DUPLEX
    (text_width, text_height), baseline = cv2.getTextSize(
        text, font, font_scale, thickness
    )

    x, y = position
    padding = 8

    # Draw background rectangle
    bg_rect = [
        (x - padding, y - text_height - padding),
        (x + text_width + padding, y + baseline + padding),
    ]
    cv2.rectangle(img, bg_rect[0], bg_rect[1], bg_color, -1)

    # Draw text
    cv2.putText(img, text, (x, y), font, font_scale, text_color, thickness)

@eaios.api
@eaios.caller
def skl_detect_objs(self_entity, camera_name: str) -> dict:
    """
    Detect all objects in the current view of the specified camera and return their categories and coordinates (in the 'map' frame).
    Args:
        camera_name: Name of the camera (e.g., 'camera')
    Returns:
        Dict: {obj_name: (x, y, z)} - Mapping from object name to 3D coordinates in the 'map' frame
    """
    try:
        # Get RGB and depth images
        rgb_image, depth_image = self_entity.cap_camera_dep_rgb(camera_name=camera_name)
        if rgb_image is None or depth_image is None:
            print("Failed to get RGB and depth images")
            return {}
        # Get camera parameters
        camera_info = self_entity.cap_camera_info(camera_name=camera_name)
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
        
        # Create visualization
        colors = create_color_palette(n_objects)
        vis_image = np_color_image.copy()
        overlay = vis_image.copy()
        cv2.rectangle(overlay, (0, 0), (vis_image.shape[1], 60), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.3, vis_image, 0.7, 0, vis_image)
        
        detected_count = 0
        print(f"YOLO detected {n_objects} objects total")
        
        for i in range(n_objects):
            name = detection.names[detection_class[i]]
            conf = detection_conf[i]
            print(f"Object {i}: {name} with confidence {conf:.3f}")
            if conf < 0.7:
                print(f"Skipping {name} due to low confidence {conf:.3f}")
                continue
                
            detected_count += 1
            print(f"detected object: [{name}] with confidence {conf}")
            
            # Get bounding box coordinates
            x1, y1, x2, y2 = object_boxes[i]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            color = colors[i % len(colors)]
            draw_rounded_rectangle(vis_image, x1, y1, x2, y2, color, thickness=3)
            
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
            map_x, map_y, map_z = self_entity.cap_tf_transform(source_frame='camera_link', target_frame='camera_link', x=world_x, y=world_y, z=center_depth)
            detected_objects[name] = (map_x, map_y, map_z)
            
            # Create comprehensive label with all information
            label = f"{name} ({conf:.4f}) | D:{center_depth:.4f}m | ({map_x:.4f}, {map_y:.4f})"
            
            text_x = x1
            text_y = max(y1 - 10, 30)
            put_text_with_background(
                vis_image,
                label,
                (text_x, text_y),
                font_scale=0.5,
                thickness=1,
                bg_color=color,
                text_color=(255, 255, 255),
            )
            
            print(f"object {name}: depth={center_depth:.3f}m, pixel_center=({center_x}, {center_y}), map_pos=({map_x:.3f}, {map_y:.3f}, {map_z:.3f})")
        
        # Add timestamp at bottom right corner
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        img_height, img_width = vis_image.shape[:2]
        
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 0.6
        thickness = 1
        (text_width, text_height), baseline = cv2.getTextSize(
            timestamp, font, font_scale, thickness
        )
        
        # Position at bottom right with padding
        padding = 10
        text_x = img_width - text_width - padding
        text_y = img_height - padding
        
        put_text_with_background(
            vis_image,
            timestamp,
            (text_x, text_y),
            font_scale=font_scale,
            thickness=thickness,
            bg_color=(30, 30, 30),
            text_color=(200, 200, 200),
        )
        
        # Save visualization
        output_dir = os.path.join(script_dir, "..", "output")
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_filename = f"detection_result_{timestamp_str}.jpg"
        output_path = os.path.join(output_dir, output_filename)
        
        cv2.imwrite(output_path, vis_image)
        print(f"detection visualization saved to: {output_path}")
        
        return detected_objects
    except Exception as e:
        print(f"Error in skl_detect_objs: {str(e)}")
        print(f"Traceback: {traceback.format_exc()}")
        return {}
