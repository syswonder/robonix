#!/bin/bash
# Simple test with complete params

source ../robonix-sdk/install/setup.bash

# Object graph with robot and objects
OBJECT_GRAPH='[{"id":"robot_001","label":"robot1","registered_skills":["navigate","pick","place","grasp","move_arm"],"registered_primitives":["navigation","manipulation"],"relations":[],"frame_mapping":[{"center":{"x":0.5,"y":0.5,"z":0.0},"bbox":[{"scale_x":0.4,"scale_y":0.4,"scale_z":0.8,"yaw":0.0}],"frame_id":"map"}]},{"id":"table_001","label":"dining_table","registered_skills":["place"],"registered_primitives":["manipulation"],"relations":[{"relation_type":{"type":0},"target_entity_id":"room_001"}],"frame_mapping":[{"center":{"x":1.0,"y":1.0,"z":0.4},"bbox":[{"scale_x":1.2,"scale_y":0.8,"scale_z":0.4,"yaw":0.0}],"frame_id":"map"}]},{"id":"box_001","label":"red_box","registered_skills":["pick"],"registered_primitives":["manipulation"],"relations":[{"relation_type":{"type":1},"target_entity_id":"table_001"}],"frame_mapping":[{"center":{"x":1.0,"y":1.0,"z":0.8},"bbox":[{"scale_x":0.2,"scale_y":0.2,"scale_z":0.2,"yaw":0.0}],"frame_id":"map"}]}]'

# RTDL syntax
RTDL_SYNTAX='{"format":"JSON array of instructions","instruction_types":["skill","primitive"],"skill_instruction":{"type":"skill","name":"string","params":{"target":"string"}},"example":[{"type":"skill","name":"pick","params":{"target":"box_001"}}]}'

# Skill/Primitive specs
SPECS='{"skills":{"pick":{"description":"Pick up an object","input":{"target":"string"},"output":{"success":"bool"}},"place":{"description":"Place an object","input":{"target":"string","destination":"string"},"output":{"success":"bool"}},"navigate":{"description":"Navigate to location","input":{"target":"string","speed":"float"},"output":{"success":"bool"}}},"primitives":{"prm::arm.move.ee":{"description":"Move end effector","input":{"pose":"geometry_msgs/msg/PoseStamped","speed":"float"},"output":{"success":"bool"}}}}'

ros2 service call /demo_service/task_plan/plan robonix_sdk/srv/PlanTask \
  "{description: 'Pick up the red box on the table', params: {keys: ['object_graph', 'rtdl_syntax', 'skill_primitive_specs'], values: ['$OBJECT_GRAPH', '$RTDL_SYNTAX', '$SPECS']}}"
