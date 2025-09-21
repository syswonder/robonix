#!/usr/bin/env python3
"""
Simple test for skl_detect_objs function
"""

import sys
import os

# Add the project root to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from skill.vision.api.api import skl_detect_objs

if __name__ == "__main__":
    print("Testing skl_detect_objs with camera_name='camera'")
    
    try:
        # Call the function
        result = skl_detect_objs("camera")
        
        print(f"Result: {result}")
        print(f"Type: {type(result)}")
        print(f"Number of objects: {len(result)}")
        
        if result:
            print("Detected objects:")
            for obj_name, coords in result.items():
                print(f"  {obj_name}: {coords}")
                
    except Exception as e:
        print(f"Error: {e}") 