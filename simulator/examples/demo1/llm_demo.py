#!/usr/bin/env python3

import sys
import os
import time
import argparse
import json
import re
from pathlib import Path
import cv2
from openai import OpenAI
from dotenv import load_dotenv

project_root = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(project_root))

project_root_parent = Path(
    __file__
).parent.parent.parent.parent.parent  # DeepEmbody root
sys.path.insert(0, str(project_root_parent))

from uapi.runtime.runtime import Runtime
from uapi.runtime.provider import SkillProvider
from uapi.graph.entity import create_root_room, create_controllable_entity
from uapi.runtime.action import set_runtime
from uapi.log import logger


def init_skill_providers(runtime: Runtime):
    local_provider = SkillProvider(
        name="local_provider",
        IP="127.0.0.1",
        skills=[
            "c_space_getpos",
            "c_space_move",
            "c_camera_rgb",
            "c_camera_dep_rgb",
            "c_camera_info",
            "c_save_rgb_image",
            "c_save_depth_image",
            "c_get_robot_pose",
        ],
    )

    runtime.registry.add_provider(local_provider)
    logger.info(f"added skill providers: {runtime.registry}")


def init_entity_graph_from_yolo(runtime: Runtime):
    logger.info("importing skills...")
    from skill import s_detect_objs, c_save_rgb_image, c_save_depth_image

    root_room = create_root_room()
    runtime.set_graph(root_room)

    robot = create_controllable_entity("robot")
    root_room.add_child(robot)

    def robot_move_impl(x, y, z):
        # from driver.sim_genesis_ranger.driver import move_to_point
        # move_to_point(x, y)  # THIS IS A FUNCTION FROM DRIVER !
        return {"success": True}

    def robot_getpos_impl():
        from driver.sim_genesis_ranger.driver import get_pose
        x, y, z, yaw = get_pose()
        return {"x": x, "y": y, "z": z}

    robot.bind_skill("c_space_move", robot_move_impl)
    robot.bind_skill("c_space_getpos", robot_getpos_impl)
    robot.bind_skill("c_save_rgb_image", c_save_rgb_image)
    robot.bind_skill("c_save_depth_image", c_save_depth_image)

    move_base = create_controllable_entity("move_base")
    robot.add_child(move_base)

    camera = create_controllable_entity("camera")
    robot.add_child(camera)

    camera.bind_skill("s_detect_objs", s_detect_objs)

    detect_objs = camera.s_detect_objs(camera_name="camera0")
    logger.info(f"detected objects: {detect_objs}")
    # detect_objs is a dict of {obj_name: obj_info}

    global detected_entities
    detected_entities = {}

    global detected_entities_getpos_handler
    detected_entities_getpos_handler = {}

    for obj_name, obj_info in detect_objs.items():
        obj_entity = create_controllable_entity(obj_name)
        root_room.add_child(obj_entity)
        detected_entities[obj_name] = obj_entity

        x, y = obj_info["position"][0], obj_info["position"][1]
        # Fix closure issue by creating a proper function with default arguments
        def create_getpos_handler(obj_x, obj_y):
            return lambda: {"x": obj_x, "y": obj_y, "z": 0.0}
        
        detected_entities_getpos_handler[obj_name] = create_getpos_handler(x, y)
        obj_entity.bind_skill(
            "c_space_getpos", detected_entities_getpos_handler[obj_name]
        )

        logger.info(f"created entity for {obj_name}: {obj_entity.get_absolute_path()}")

    logger.info("initd entity graph from YOLO detection:")
    logger.info(f"  root room: {root_room.get_absolute_path()}")
    logger.info(f"  robot: {robot.get_absolute_path()}")
    logger.info(f"  detected entities: {list(detected_entities.keys())}")

    return detected_entities


def collect_entity_graph_info(runtime: Runtime, detected_entities: dict):
    """Collect entity graph structure information and bound skills for each entity"""
    graph_info = {
        "entities": {},
        "skills": {},
        "graph_structure": {}
    }
    
    # Get all entity information
    root_room = runtime.get_graph()
    
    def collect_entity_info(entity, parent_path=""):
        entity_path = entity.get_absolute_path()
        entity_name = entity.entity_name
        
        # Collect basic entity information
        graph_info["entities"][entity_path] = {
            "name": entity_name,
            "parent": parent_path,
            "children": []
        }
        
        # Collect bound skill information
        bound_skills = entity.primitives
        graph_info["skills"][entity_path] = bound_skills
        
        # Recursively process child entities
        for child in entity.get_children():
            child_path = child.get_absolute_path()
            graph_info["entities"][entity_path]["children"].append(child_path)
            collect_entity_info(child, entity_path)
    
    collect_entity_info(root_room)
    
    # Build hierarchical graph structure
    def build_graph_structure(entity_path):
        entity_info = graph_info["entities"][entity_path]
        structure = {
            "name": entity_info["name"],
            "path": entity_path,
            "skills": graph_info["skills"][entity_path],
            "children": {}
        }
        
        for child_path in entity_info["children"]:
            child_name = graph_info["entities"][child_path]["name"]
            structure["children"][child_name] = build_graph_structure(child_path)
        
        return structure
    
    graph_info["graph_structure"] = build_graph_structure(root_room.get_absolute_path())
    
    return graph_info


def load_skill_specs():
    """Load skill standard interface definitions"""
    from uapi.specs.skill_specs import EOS_SKILL_SPECS
    return EOS_SKILL_SPECS


def load_demo_action_example():
    """Load demo action syntax and file format example"""
    # Load action example from the same directory as the running Python file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    action_example_path = os.path.join(current_dir, "simple.action")
    with open(action_example_path, 'r', encoding='utf-8') as f:
        return f.read()


def create_llm_prompt(entity_graph_info: dict, skill_specs: dict, action_example: str, user_request: str):
    """Create structured prompt to send to LLM"""
    
    # Build skill descriptions
    skills_description = {}
    for entity_path, skills in entity_graph_info["skills"].items():
        for skill_name in skills:
            if skill_name in skill_specs:
                skill_info = skill_specs[skill_name]
                # Convert any non-serializable objects to strings
                input_spec = skill_info["input"]
                output_spec = skill_info["output"]
                
                # Handle input/output that might be classes or complex types
                if input_spec is not None and not isinstance(input_spec, (dict, list, str, int, float, bool)):
                    input_spec = str(input_spec)
                if output_spec is not None and not isinstance(output_spec, (dict, list, str, int, float, bool)):
                    output_spec = str(output_spec)
                
                # Ensure all values are JSON serializable
                try:
                    # Test JSON serialization of the skill info
                    test_dict = {
                        "description": skill_info["description"],
                        "input": input_spec,
                        "output": output_spec,
                        "type": skill_info["type"].value
                    }
                    json.dumps(test_dict)  # This will fail if any value is not serializable
                    skills_description[f"{entity_path}.{skill_name}"] = test_dict
                except (TypeError, ValueError):
                    # Fallback: convert everything to strings for JSON compatibility
                    skills_description[f"{entity_path}.{skill_name}"] = {
                        "description": str(skill_info["description"]),
                        "input": str(input_spec) if input_spec is not None else None,
                        "output": str(output_spec) if output_spec is not None else None,
                        "type": str(skill_info["type"].value)
                    }
    
    prompt = f"""You are a robot task planning expert. Based on the following information, generate a Python action function for the user's natural language requirement.

## Current Scene Information

### Entity Graph Structure
```json
{json.dumps(entity_graph_info["graph_structure"], indent=2, ensure_ascii=False)}
```

### Available Skills
```json
{json.dumps(skills_description, indent=2, ensure_ascii=False)}
```

### Action Syntax Example
```python
{action_example}
```

## User Requirement
{user_request}

## Task Requirements
1. Analyze user requirements and understand the specific task to be executed
2. Design a reasonable execution action based on available entities and skills
3. Generate a complete Python action function with a descriptive function name
4. Specify action function parameter values, including:
   - entity_path parameters: use actual paths like "/robot", "/chair", etc.
   - other parameters: use specific values or strings

## Output Format
Please output strictly in the following JSON format:

```json
{{
    "action_code": "Complete Python action function code with @action decorator and function definition",
    "action_args": {{
        "parameter_name": "parameter_value",
        "parameter_name": "parameter_value"
    }},
    "task_description": "Detailed description of task execution steps"
}}
```

Notes:
- action_code must be complete, executable Python code
- Use @action decorator
- Function parameter type annotations should use EntityPath
- Return value should use EOS_TYPE_ActionResult.SUCCESS or EOS_TYPE_ActionResult.FAILURE
- Use action_print() for log output
- Ensure all used skills are actually bound to entities
"""

    return prompt


def call_deepseek_llm(prompt: str, api_key: str):
    """Call DeepSeek LLM API"""
    try:
        client = OpenAI(
            base_url="https://api.deepseek.com",
            api_key=api_key,
        )
        
        response = client.chat.completions.create(
            model="deepseek-chat",
            messages=[
                {"role": "system", "content": "You are a professional robot task planning expert."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.1
        )
        
        return response.choices[0].message.content
    except Exception as e:
        logger.error(f"Failed to call DeepSeek LLM: {str(e)}")
        return None


def extract_llm_response(llm_response: str):
    """Extract action code and parameters from LLM response"""
    try:
        # Find JSON code block
        json_pattern = r'```json\s*\n(.*?)\n```'
        json_match = re.search(json_pattern, llm_response, re.DOTALL)
        
        if json_match:
            json_str = json_match.group(1)
            parsed_response = json.loads(json_str)
            
            action_code = parsed_response.get("action_code", "")
            action_args = parsed_response.get("action_args", {})
            task_description = parsed_response.get("task_description", "")
            
            return action_code, action_args, task_description
        else:
            logger.error("No JSON format code block found in LLM response")
            return None, None, None
            
    except Exception as e:
        logger.error(f"Failed to parse LLM response: {str(e)}")
        return None, None, None


def save_debug_files(entity_graph_info: dict, skill_specs: dict, action_example: str, 
                    user_request: str, llm_response: str, action_code: str, action_args: dict):
    """Save debug information to local files"""
    # Save files in the same directory as the running Python file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    debug_dir = os.path.join(current_dir, "llm_demo_debug")
    os.makedirs(debug_dir, exist_ok=True)
    
    # Convert skill_specs to JSON-serializable format
    serializable_skill_specs = {}
    for skill_name, skill_info in skill_specs.items():
        serializable_skill_specs[skill_name] = {
            "description": str(skill_info["description"]),
            "type": str(skill_info["type"].value),  # Convert enum to string
            "input": str(skill_info["input"]) if skill_info["input"] is not None else None,
            "output": str(skill_info["output"]) if skill_info["output"] is not None else None,
            "dependencies": skill_info.get("dependencies", [])
        }
    
    # Save structured data
    structured_data = {
        "entity_graph_info": entity_graph_info,
        "skill_specs": serializable_skill_specs,
        "action_example": action_example,
        "user_request": user_request,
        "llm_response": llm_response,
        "generated_action_code": action_code,
        "action_args": action_args,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
    }
    
    with open(os.path.join(debug_dir, "debug_data.json"), 'w', encoding='utf-8') as f:
        json.dump(structured_data, f, indent=2, ensure_ascii=False)
    
    # Save generated action code
    if action_code:
        with open(os.path.join(debug_dir, "generated_action.py"), 'w', encoding='utf-8') as f:
            f.write(action_code)
    
    logger.info(f"Debug information saved to: {debug_dir}")


def execute_generated_action(runtime: Runtime, action_code: str, action_args: dict):
    """Execute LLM generated action"""
    try:
        # Create temporary action file in the same directory as the running Python file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        temp_action_path = os.path.join(current_dir, "tmp.action")
        with open(temp_action_path, 'w', encoding='utf-8') as f:
            f.write(action_code)
        
        # Load action program
        action_names = runtime.load_program(temp_action_path)
        logger.info(f"Loaded action functions: {action_names}")
        
        if not action_names:
            logger.error("No executable action function found")
            return False
        
        # Get first action function name
        action_name = action_names[0]
        
        # Set action arguments
        runtime.set_action_args(action_name, **action_args)
        
        # Execute action
        logger.info(f"Starting action execution: {action_name}")
        logger.info(f"Action arguments: {action_args}")
        
        # Start the specific action
        runtime.start_action(action_name)
        
        # Wait for completion
        results = runtime.wait_for_all_actions(timeout=30.0)
        
        logger.info("Action execution results:")
        for name, result in results.items():
            logger.info(f"  {name}: {result}")
        
        # Clean up temporary file
        os.remove(temp_action_path)
        
        return True
        
    except Exception as e:
        logger.error(f"Failed to execute generated action: {str(e)}", exc_info=True)
        return False


def main():
    # Load environment variables from .env file in the same directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    env_path = os.path.join(current_dir, ".env")
    load_dotenv(env_path)
    
    # Get API key from environment variable
    api_key = os.getenv("DEEPSEEK_API_KEY")
    if not api_key:
        logger.error("DEEPSEEK_API_KEY not found in .env file")
        return 1
    
    logger.info("Starting LLM task planning demo")

    # Initialize runtime
    runtime = Runtime()
    init_skill_providers(runtime)
    
    # Build entity graph
    detected_entities = init_entity_graph_from_yolo(runtime)
    set_runtime(runtime)
    
    # Collect scene information
    entity_graph_info = collect_entity_graph_info(runtime, detected_entities)
    skill_specs = load_skill_specs()
    action_example = load_demo_action_example()
    
    # Wait for user input
    print("\n" + "="*50)
    print("Scene construction completed! Entity Graph Structure:")
    print("="*50)
    
    def print_entity_tree(entity, prefix="", is_last=True):
        """Recursively print entity tree structure with skills"""
        entity_path = entity.get_absolute_path()
        entity_name = entity.entity_name
        
        # ANSI color codes
        RESET = "\033[0m"
        BOLD = "\033[1m"
        BLUE = "\033[94m"      # Entity name color
        GREEN = "\033[92m"     # Skills color
        YELLOW = "\033[93m"    # Path color
        GRAY = "\033[90m"      # No skills color
        
        # Print current entity with skills on the same line
        connector = "└── " if is_last else "├── "
        if entity.primitives:
            skills_str = f" {GREEN}[skills: {', '.join(entity.primitives)}]{RESET}"
        else:
            skills_str = f" {GRAY}[no skills]{RESET}"
        
        print(f"{prefix}{connector}{BOLD}{BLUE}{entity_name}{RESET} {YELLOW}({entity_path}){RESET}{skills_str}")
        
        # Recursively print children
        children = entity.get_children()
        for i, child in enumerate(children):
            is_last_child = i == len(children) - 1
            print_entity_tree(child, prefix + ("    " if is_last else "│   "), is_last_child)
    
    # Print the complete entity tree
    root_entity = runtime.get_graph()
    print_entity_tree(root_entity)
    
    print("="*50)
    print("Detected objects from YOLO:")
    for entity_name in detected_entities.keys():
        print(f"  - {entity_name}")
    print("="*50)
    
    user_request = input("\nPlease enter your command: ")
    
    if not user_request:
        logger.info("User did not input command, exiting program")
        return 0
    
    # Create LLM prompt
    prompt = create_llm_prompt(entity_graph_info, skill_specs, action_example, user_request)
    
    # Save prompt to local file for debugging
    current_dir = os.path.dirname(os.path.abspath(__file__))
    prompt_file = os.path.join(current_dir, "llm_prompt.txt")
    with open(prompt_file, 'w', encoding='utf-8') as f:
        f.write(prompt)
    logger.info(f"LLM prompt saved to: {prompt_file}")
    
    # Call LLM
    logger.info("Calling DeepSeek LLM for task planning...")
    start_time = time.time()
    llm_response = call_deepseek_llm(prompt, api_key)
    end_time = time.time()
    llm_time = end_time - start_time
    logger.info(f"LLM response received in {llm_time:.2f} seconds")
    
    if not llm_response:
        logger.error("LLM call failed")
        return 1
    
    # Parse LLM response
    action_code, action_args, task_description = extract_llm_response(llm_response)
    
    if not action_code or not action_args:
        logger.error("Unable to extract action code or parameters from LLM response")
        return 1
    
    # Save debug information
    save_debug_files(entity_graph_info, skill_specs, action_example, 
                    user_request, llm_response, action_code, action_args)
    
    # Display task description
    print(f"\nTask planning result:")
    print(f"Task description: {task_description}")
    print(f"Action parameters: {action_args}")
    
    # Ask whether to execute
    execute = input("\nDo you want to execute this task? (y/n): ").strip().lower()
    
    if execute in ['y', 'yes']:
        logger.info("Starting execution of LLM generated action...")
        success = execute_generated_action(runtime, action_code, action_args)
        
        if success:
            logger.info("Task execution completed!")
        else:
            logger.error("Task execution failed")
            return 1
    else:
        logger.info("User chose not to execute task")
    
    logger.info("LLM demo completed")
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
