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

project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

project_root_parent = Path(
    __file__
).parent.parent.parent.parent  # robonix root
sys.path.insert(0, str(project_root_parent))

from robonix.manager.log import logger
from robonix.uapi import get_runtime, set_runtime

def init_skill_providers(runtime):
    """Initialize skill providers"""
    from robonix.uapi.runtime.provider import SkillProvider

    # dump __all__ in robonix.skill to skills list
    try:
        from robonix.skill import __all__
        skills = __all__
    except ImportError:
        logger.warning("robonix.skill module not available")
        skills = []

    local_provider = SkillProvider(
        name="local_provider",
        IP="127.0.0.1",
        skills=skills,
    )

    runtime.registry.add_provider(local_provider)
    logger.info(f"Added skill providers: {runtime.registry}")


def create_yolo_entity_builder():
    """Create a YOLO-based entity graph builder"""
    def builder(runtime, **kwargs):
        logger.info("Building entity graph from YOLO detection...")

        try:
            from robonix.skill import (
                sim_skl_detect_objs,
                sim_save_rgb_image,
                sim_save_depth_image,
                sim_camera_dep_rgb,
                sim_camera_info,
                sim_get_robot_pose,
            )
        except ImportError:
            logger.error("Required skills not available")
            return

        from robonix.uapi.graph.entity import create_root_room, create_controllable_entity

        root_room = create_root_room()
        runtime.set_graph(root_room)

        robot = create_controllable_entity("robot")
        root_room.add_child(robot)

        def robot_move_impl(x, y, z):
            try:
                from robonix.driver.sim_genesis_ranger.driver import move_to_point
                # move_to_point(x, y)  # Uncomment when driver is available
                return {"success": True}
            except ImportError:
                logger.warning(
                    "Driver module not available, using mock implementation")
                return {"success": True}

        def robot_getpos_impl():
            try:
                from robonix.driver.sim_genesis_ranger.driver import get_pose
                x, y, z, yaw = get_pose()
                return {"x": x, "y": y, "z": z}
            except ImportError:
                logger.warning(
                    "Driver module not available, using mock implementation")
                return {"x": 0.0, "y": 0.0, "z": 0.0}

        # Bind skills to robot entity
        robot.bind_skill("cap_space_move", robot_move_impl)
        robot.bind_skill("cap_space_getpos", robot_getpos_impl)
        robot.bind_skill("cap_save_rgb_image", sim_save_rgb_image)
        robot.bind_skill("cap_save_depth_image", sim_save_depth_image)

        move_base = create_controllable_entity("move_base")
        robot.add_child(move_base)

        camera = create_controllable_entity("camera")
        robot.add_child(camera)

        # Bind camera capabilities
        camera.bind_skill("cap_camera_dep_rgb", sim_camera_dep_rgb)
        camera.bind_skill("cap_camera_info", sim_camera_info)
        camera.bind_skill("cap_get_robot_pose", sim_get_robot_pose)
        camera.bind_skill("skl_detect_objs", sim_skl_detect_objs)

        # Detect objects
        detect_objs = camera.skl_detect_objs(camera_name="camera0")
        logger.info(f"Detected objects: {detect_objs}")

        detected_entities_getpos_handler = {}

        for obj_name, obj_info in detect_objs.items():
            obj_entity = create_controllable_entity(obj_name)
            root_room.add_child(obj_entity)

            if obj_info["position"] is None:
                logger.warning(f"Object {obj_name} has no position")
                x, y = 0.0, 0.0
            else:
                x, y = obj_info["position"][0], obj_info["position"][1]

            def create_getpos_handler(obj_x, obj_y):
                return lambda: {"x": obj_x, "y": obj_y, "z": 0.0}

            detected_entities_getpos_handler[obj_name] = create_getpos_handler(
                x, y)
            obj_entity.bind_skill(
                "cap_space_getpos", detected_entities_getpos_handler[obj_name]
            )

            logger.info(
                f"Created entity for {obj_name}: {obj_entity.get_absolute_path()}")

        logger.info("YOLO-based entity graph initialized:")
        logger.info(f"  root room: {root_room.get_absolute_path()}")
        logger.info(f"  robot: {robot.get_absolute_path()}")
        logger.info(f"  detected entities: {list(detect_objs.keys())}")

    return builder


def load_demo_action_example():
    """Load demo action syntax and file format example"""
    # Load action example from the same directory as the running Python file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    action_example_path = os.path.join(current_dir, "simple.action")
    with open(action_example_path, "r", encoding="utf-8") as f:
        return f.read()


def create_llm_prompt(scene_info: dict, action_example: str, user_request: str):
    """Create structured prompt to send to LLM"""

    # Build skill descriptions
    skills_description = {}
    for entity_path, skills in scene_info["entity_graph"]["skills"].items():
        for skill_name in skills:
            if skill_name in scene_info["skill_specs"]["skill_specs"]:
                skill_info = scene_info["skill_specs"]["skill_specs"][skill_name]
                skills_description[f"{entity_path}.{skill_name}"] = skill_info

    prompt = f"""You are a robot task planning expert. Based on the following information, generate a Python action function for the user's natural language requirement.

## Current Scene Information

### Entity Graph Structure
```json
{json.dumps(scene_info["entity_graph"]["graph_structure"], indent=2, ensure_ascii=False)}
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
                {
                    "role": "system",
                    "content": "You are a professional robot task planning expert.",
                },
                {"role": "user", "content": prompt},
            ],
            temperature=0.1,
        )

        return response.choices[0].message.content
    except Exception as e:
        logger.error(f"Failed to call DeepSeek LLM: {str(e)}")
        return None


def extract_llm_response(llm_response: str):
    """Extract action code and parameters from LLM response"""
    try:
        # Find JSON code block - updated regex to handle multi-line JSON properly
        json_pattern = r"```json\s*\n(.*?)\n```"
        json_match = re.search(json_pattern, llm_response, re.DOTALL)

        if json_match:
            json_str = json_match.group(1).strip()
            logger.debug(f"Extracted JSON string: {json_str}")
            parsed_response = json.loads(json_str)

            action_code = parsed_response.get("action_code", "")
            action_args = parsed_response.get("action_args", {})
            task_description = parsed_response.get("task_description", "")

            return action_code, action_args, task_description
        else:
            logger.error("No JSON format code block found in LLM response")
            logger.debug(f"LLM response content: {llm_response}")
            return None, None, None

    except Exception as e:
        logger.error(f"Failed to parse LLM response: {str(e)}")
        logger.debug(f"LLM response content: {llm_response}")
        return None, None, None


def save_debug_files(scene_info: dict, action_example: str, user_request: str,
                     llm_response: str, action_code: str, action_args: dict):
    """Save debug information to local files"""
    # Save files in the same directory as the running Python file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    debug_dir = os.path.join(current_dir, "llm_demo_debug")
    os.makedirs(debug_dir, exist_ok=True)

    # Save structured data
    structured_data = {
        "scene_info": scene_info,
        "action_example": action_example,
        "user_request": user_request,
        "llm_response": llm_response,
        "generated_action_code": action_code,
        "action_args": action_args,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    with open(os.path.join(debug_dir, "debug_data.json"), "w", encoding="utf-8") as f:
        json.dump(structured_data, f, indent=2, ensure_ascii=False)

    # Save generated action code
    if action_code:
        with open(
            os.path.join(debug_dir, "generated_action.py"), "w", encoding="utf-8"
        ) as f:
            f.write(action_code)

    logger.info(f"Debug information saved to: {debug_dir}")


def execute_generated_action(runtime, action_code: str, action_args: dict):
    """Execute LLM generated action"""
    try:
        # Create temporary action file in the same directory as the running Python file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        temp_action_path = os.path.join(current_dir, "tmp.action")
        with open(temp_action_path, "w", encoding="utf-8") as f:
            f.write(action_code)

        # Load action program
        action_names = runtime.load_action_program(temp_action_path)
        logger.info(f"Loaded action functions: {action_names}")

        if not action_names:
            logger.error("No executable action function found")
            return False

        # Get first action function name
        action_name = action_names[0]

        # Configure action arguments
        runtime.configure_action(action_name, **action_args)

        # Execute action
        logger.info(f"Starting action execution: {action_name}")
        logger.info(f"Action arguments: {action_args}")

        # Start the specific action
        thread = runtime.start_action(action_name)
        result = runtime.wait_for_action(action_name, timeout=30.0)
        logger.info(f"Action execution result: {result}")

        # Clean up temporary file
        os.remove(temp_action_path)

        return True

    except Exception as e:
        logger.error(
            f"Failed to execute generated action: {str(e)}", exc_info=True)
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

    # Get runtime instance
    runtime = get_runtime()
    init_skill_providers(runtime)

    # Register entity builders
    runtime.register_entity_builder("yolo", create_yolo_entity_builder())

    # Build entity graph using YOLO detection
    runtime.build_entity_graph("yolo")
    set_runtime(runtime)

    # Export scene information
    scene_info = runtime.export_scene_info()
    action_example = load_demo_action_example()

    # Print entity tree structure
    runtime.print_entity_tree()

    user_request = input("\nPlease enter your command: ")

    if not user_request:
        logger.info("User did not input command, exiting program")
        return 0

    # Create LLM prompt
    prompt = create_llm_prompt(scene_info, action_example, user_request)

    # Save prompt to local file for debugging
    current_dir = os.path.dirname(os.path.abspath(__file__))
    prompt_file = os.path.join(current_dir, "llm_prompt.txt")
    with open(prompt_file, "w", encoding="utf-8") as f:
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

    # save raw response to local file for debugging
    with open(os.path.join(current_dir, "llm_response.txt"), "w", encoding="utf-8") as f:
        f.write(llm_response)
    logger.info(f"LLM response saved to: {os.path.join(current_dir, "llm_response.txt")}")

    action_code, action_args, task_description = extract_llm_response(
        llm_response)
    
    # Save debug information
    save_debug_files(
        scene_info,
        action_example,
        user_request,
        llm_response,
        action_code,
        action_args,
    )

    if not action_code or action_args is None:
        logger.error(
            "Unable to extract action code or parameters from LLM response")
        return 1


    # Display task description
    print(f"\nTask planning result:")
    print(f"Task description: {task_description}")
    print(f"Action parameters: {action_args}")

    # Ask whether to execute
    execute = input(
        "\nDo you want to execute this task? (y/n): ").strip().lower()

    if execute in ["y", "yes"]:
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
