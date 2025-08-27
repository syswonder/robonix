#!/usr/bin/env python3
"""
Test script for semantic map skills
"""

import sys
import os

# Add the project root to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from skill.semantic_map.api.api import skl_update_map, skl_query_map_all, skl_query_map, skl_add_map_obj

def test_semantic_map():
    print("Testing semantic map skills...")
    
    # Test 1: Add objects manually
    print("\n1. Testing skl_add_map_obj...")
    success1 = skl_add_map_obj("cup", (1.0, 2.0, 0.5))
    success2 = skl_add_map_obj("bottle", (2.0, 1.0, 0.3))
    print(f"Added cup: {success1}")
    print(f"Added bottle: {success2}")
    
    # Test 2: Query specific object
    print("\n2. Testing skl_query_map...")
    cup_coords = skl_query_map("cup")
    bottle_coords = skl_query_map("bottle")
    non_existent = skl_query_map("table")
    print(f"Cup coordinates: {cup_coords}")
    print(f"Bottle coordinates: {bottle_coords}")
    print(f"Non-existent object: {non_existent}")
    
    # Test 3: Query all objects
    print("\n3. Testing skl_query_map_all...")
    all_objects = skl_query_map_all()
    print(f"All objects in map: {all_objects}")
    
    # Test 4: Update map (this will require camera and skl_detect_objs to work)
    print("\n4. Testing skl_update_map...")
    print("Note: This test requires camera access and skl_detect_objs skill to be available")
    try:
        update_success = skl_update_map("camera")
        print(f"Update map result: {update_success}")
    except Exception as e:
        print(f"Update map failed (expected if camera not available): {e}")
    
    print("\nSemantic map test completed!")

if __name__ == "__main__":
    test_semantic_map() 