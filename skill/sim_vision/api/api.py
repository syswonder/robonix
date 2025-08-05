from manager.eaios_decorators import eaios
# from skill import c_camera_dep_rgb, c_camera_info # if you want to temporarily let LSP to parse the symbols, uncomment this, for other scenarios, comment it! - wheatfox
from manager.log import logger
import numpy as np
import os
from ultralytics import YOLOE
import cv2
import random
from datetime import datetime

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
            label = f"{name} ({conf:.2f})"
            text_x = x1
            text_y = max(y1 - 10, 30)
            put_text_with_background(vis_image, label, (text_x, text_y), 
                                   font_scale=0.7, thickness=2,
                                   bg_color=color, text_color=(255, 255, 255))
            detected_objects[name] = {
                'confidence': float(conf),
                'bbox': [x1, y1, x2, y2]
            }
        
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
        logger.info(f"Detection visualization saved to: {output_path}")
        
        return detected_objects
    except Exception as e:
        logger.error(f"error in s_detect_objs: {e}")
        return {}