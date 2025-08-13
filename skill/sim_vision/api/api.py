from manager.eaios_decorators import eaios
# from skill import c_camera_dep_rgb, c_camera_info # if you want to temporarily let LSP to parse the symbols, uncomment this, for other scenarios, comment it! - wheatfox
from manager.log import logger
import numpy as np
import os
from ultralytics import YOLOE
import cv2
import random
from datetime import datetime
import math

from skill.vision.api.vision import px2xy, remove_mask_outliers, get_mask_center_opencv

def create_color_palette(n_colors):
    """Create a beautiful color palette for object detection"""
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
    """Draw a rounded rectangle with gradient effect"""
    # Draw main rectangle
    cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
    
    # Draw rounded corners
    cv2.circle(img, (x1 + radius, y1 + radius), radius, color, -1)
    cv2.circle(img, (x2 - radius, y1 + radius), radius, color, -1)
    cv2.circle(img, (x1 + radius, y2 - radius), radius, color, -1)
    cv2.circle(img, (x2 - radius, y2 - radius), radius, color, -1)

def put_text_with_background(img, text, position, font_scale=0.6, thickness=2, bg_color=(0, 0, 0), text_color=(255, 255, 255)):
    """Put text with a beautiful background"""
    font = cv2.FONT_HERSHEY_DUPLEX
    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    
    x, y = position
    padding = 8
    
    # Draw background rectangle
    bg_rect = [
        (x - padding, y - text_height - padding),
        (x + text_width + padding, y + baseline + padding)
    ]
    cv2.rectangle(img, bg_rect[0], bg_rect[1], bg_color, -1)
    
    # Draw text
    cv2.putText(img, text, (x, y), font, font_scale, text_color, thickness)

@eaios.api
@eaios.caller
def s_detect_objs(camera_name: str) -> dict:
    # this is used for simulation environment
    try:
        rgb_image, depth_image = c_camera_dep_rgb(camera_name)
        if rgb_image is None or depth_image is None:
            logger.error("failed to get RGB and depth images")
            return {}
        
        camera_info = c_camera_info(camera_name)
        if camera_info is None:
            logger.error("failed to get camera info")
            return {}
        
        np_color_image = np.array(rgb_image, dtype=np.uint8)
        np_depth_image = np.array(depth_image, dtype=np.uint16)
        
        if np_color_image is None:
            return {}
        
        # Load YOLO model
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, "..", "models", "yoloe-11l-seg-pf.pt")
        detector = YOLOE(model_path)
        results = detector(source=np_color_image)
        
        detection = results[0]
        if detection is None:
            return {}
        
        object_boxes = detection.boxes.xyxy.cpu().numpy()
        n_objects = object_boxes.shape[0]
        masks = detection.masks.cpu()
        detection_class = detection.boxes.cls.cpu().numpy()
        detection_conf = detection.boxes.conf.cpu().numpy()
        detected_objects = {}
        
        colors = create_color_palette(n_objects)
        vis_image = np_color_image.copy()
        overlay = vis_image.copy()
        cv2.rectangle(overlay, (0, 0), (vis_image.shape[1], 60), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.3, vis_image, 0.7, 0, vis_image)
        
        detected_count = 0
        for i in range(n_objects):
            name = detection.names[detection_class[i]]
            conf = detection_conf[i]
            if conf < 0.7:
                continue
                
            detected_count += 1
            logger.info(f"detected object: {name} with confidence {conf}")
            
            # Get bounding box coordinates
            x1, y1, x2, y2 = object_boxes[i]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            color = colors[i % len(colors)]
            draw_rounded_rectangle(vis_image, x1, y1, x2, y2, color, thickness=3)
            
            # Enhanced object detection with depth information and coordinate transformation
            try:
                # Get mask center for depth estimation
                mask_points = masks.xy[i].reshape(-1, 1, 2).astype(np.int32)
                center_x, center_y = get_mask_center_opencv(mask_points)
                single_selection_mask = np.array(masks.xy[i])
                
                # Improved depth computation using mask area
                depths = []
                valid_depth_count = 0
                
                # Sample points from the mask for depth estimation
                sample_points = single_selection_mask[::max(1, len(single_selection_mask) // 50)]
                
                for point in sample_points:
                    p_x = int(point[0])
                    p_y = int(point[1])
                    if 0 <= p_x < np_depth_image.shape[1] and 0 <= p_y < np_depth_image.shape[0]:
                        depth = np_depth_image.item(p_y, p_x)
                        # Filter out invalid depth values (0 or NaN)
                        if depth > 0 and depth == depth:  # depth == depth checks for NaN
                            depths.append(depth)
                            valid_depth_count += 1
                
                # Use robust depth estimation - fix depth calculation
                center_depth = 0
                if valid_depth_count > 0:
                    # Remove outliers and get median depth for stability
                    filtered_depths = remove_mask_outliers(depths, lower_percentile=20, upper_percentile=80)
                    if len(filtered_depths) > 0:
                        # Use median depth for more robust estimation
                        center_depth = np.median(filtered_depths)  # Keep in original units (mm)
                    else:
                        center_depth = np.median(depths)  # Keep in original units (mm)
                else:
                    # Fallback: try to get depth at center point
                    if 0 <= center_x < np_depth_image.shape[1] and 0 <= center_y < np_depth_image.shape[0]:
                        center_depth = np_depth_image[int(center_y), int(center_x)]
                        if center_depth == 0 or center_depth != center_depth:  # Check for NaN
                            center_depth = 0
                
                # Skip objects with invalid depth
                if center_depth <= 0:
                    logger.warning(f"skipping object {name} due to invalid depth: {center_depth}")
                    detected_objects[name] = {
                        'confidence': float(conf),
                        'bbox': [x1, y1, x2, y2],
                        'depth': None,
                        'position': None,
                        'camera_position': None
                    }
                    
                    # All information will be combined in the label text (handled later)
                    
                    continue
                
                # Convert depth to meters for coordinate calculation
                center_depth_m = center_depth / 1000.0
                
                # Convert pixel to camera coordinates using the computed depth
                world_x, world_y = px2xy([center_x, center_y], camera_info["k"], camera_info["d"], center_depth_m)
                
                # Use c_tf_transform to convert to 'map' frame
                map_x, map_y, map_z = c_tf_transform('camera_link', 'map', world_x, world_y, center_depth_m)
                
                # Store enhanced object information
                detected_objects[name] = {
                    'confidence': float(conf),
                    'bbox': [x1, y1, x2, y2],
                    'depth': float(center_depth_m),
                    'position': (float(map_x), float(map_y), float(map_z)),
                    'camera_position': (float(world_x), float(world_y), float(center_depth_m)),
                    'pixel_center': (int(center_x), int(center_y))
                }
                
                # All information will be combined in the label text (handled later)
                
                logger.info(f"object {name}: depth={center_depth_m:.3f}m, "
                           f"camera_pos=({world_x:.3f}, {world_y:.3f}, {center_depth_m:.3f}), "
                           f"map_pos=({map_x:.3f}, {map_y:.3f}, {map_z:.3f})")
                
            except Exception as e:
                logger.error(f"error processing depth for object {name}: {e}")
                # Fallback to basic detection without depth
                detected_objects[name] = {
                    'confidence': float(conf),
                    'bbox': [x1, y1, x2, y2],
                    'depth': None,
                    'position': None,
                    'camera_position': None
                }
                
                # All information will be combined in the label text (handled later)
            
            # Create comprehensive label with all information
            if detected_objects[name].get('depth') is not None:
                # Object has valid depth and position
                label = f"{name} ({conf:.2f}) | D:{detected_objects[name]['depth']:.2f}m | ({detected_objects[name]['position'][0]:.2f}, {detected_objects[name]['position'][1]:.2f})"
            else:
                # Object has no depth data
                label = f"{name} ({conf:.2f}) | No Depth Data"
            
            text_x = x1
            text_y = max(y1 - 10, 30)
            put_text_with_background(vis_image, label, (text_x, text_y), 
                                   font_scale=0.5, thickness=1,
                                   bg_color=color, text_color=(255, 255, 255))
        
        # Add timestamp at bottom right corner
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        img_height, img_width = vis_image.shape[:2]
        
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 0.6
        thickness = 1
        (text_width, text_height), baseline = cv2.getTextSize(timestamp, font, font_scale, thickness)
        
        # Position at bottom right with padding
        padding = 10
        text_x = img_width - text_width - padding
        text_y = img_height - padding
        
        put_text_with_background(vis_image, timestamp, (text_x, text_y), 
                               font_scale=font_scale, thickness=thickness,
                               bg_color=(30, 30, 30), text_color=(200, 200, 200))
        
        output_dir = os.path.join(script_dir, "..", "output")
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_filename = f"detection_result_{timestamp_str}.jpg"
        output_path = os.path.join(output_dir, output_filename)
        
        cv2.imwrite(output_path, vis_image)
        logger.info(f"detection visualization saved to: {output_path}")
        
        return detected_objects
    except Exception as e:
        logger.error(f"error in s_detect_objs: {e}")
        return {}