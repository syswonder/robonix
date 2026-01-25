#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Task Plan Service
#
# Demo task plan service implementation.
# Converts natural language task description to RTDL code.
# Uses DeepSeek LLM API for intelligent task planning.
# Uses simple list-style RTDL format (assembly-like instruction list).

import os
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from robonix_sdk.srv import PlanTask
from builtin_interfaces.msg import Time
from dotenv import load_dotenv
from openai import OpenAI


class TaskPlanService(Node):
    """Implements task_plan service with DeepSeek LLM for RTDL generation."""

    def __init__(self):
        super().__init__('demo_task_plan_service')
        
        # Load environment variables from .env file in package root directory
        # Find package root directory (containing setup.py) by walking up from current file
        current_file = Path(__file__).resolve()
        package_root = current_file.parent
        # Walk up directories to find package root (containing setup.py)
        while package_root != package_root.parent:
            if (package_root / 'setup.py').exists():
                break
            package_root = package_root.parent
        # If setup.py not found, use current file's parent's parent as fallback
        if not (package_root / 'setup.py').exists():
            package_root = current_file.parent.parent
        
        env_path = package_root / '.env'
        load_dotenv(env_path)
        
        # Get DeepSeek API key from environment
        self.api_key = os.getenv('DEEPSEEK_API_KEY')
        if not self.api_key:
            self.get_logger().error(
                'DEEPSEEK_API_KEY not found in .env file. '
                'Please configure DEEPSEEK_API_KEY in .env file. '
                'See README.md for configuration instructions.'
            )
            raise ValueError(
                'DEEPSEEK_API_KEY not found. '
                'Please configure DEEPSEEK_API_KEY in .env file.'
            )
        
        # Validate API key format
        if not self._validate_api_key(self.api_key):
            self.get_logger().error(
                'Invalid DEEPSEEK_API_KEY format. '
                'API key should start with "sk-" and be at least 35 characters long. '
                'Please check your .env file.'
            )
            raise ValueError(
                'Invalid DEEPSEEK_API_KEY format. '
                'API key should start with "sk-" and be at least 35 characters long.'
            )
        
        # Initialize DeepSeek client
        try:
            self.deepseek_client = OpenAI(
                base_url="https://api.deepseek.com",
                api_key=self.api_key,
            )
            self.get_logger().info(f'DeepSeek API client initialized with key: {self.api_key[:10]}...')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize DeepSeek API client: {e}')
            raise
        
        # Create service
        self.service = self.create_service(
            PlanTask,
            '/demo_service/task_plan/plan',
            self.plan_callback
        )
        
        self.get_logger().info('Task plan service started')
        self.get_logger().info('  Service: /demo_service/task_plan/plan')

    def plan_callback(self, request, response):
        """Handle task planning request."""
        self.get_logger().info('=' * 80)
        self.get_logger().info('Task Plan Service: Received planning request')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'Task Description: {request.description}')
        self.get_logger().info(f'Params keys: {request.params.keys if request.params else []}')
        
        # Parse params to extract object graph, RTDL syntax, and skill/primitive specs
        object_graph = None
        rtdl_syntax = None
        skill_primitive_specs = None
        skill_specs = None
        
        if request.params:
            # Extract all params from Dict (keys and values arrays)
            for i, key in enumerate(request.params.keys):
                if i < len(request.params.values):
                    try:
                        if key == 'object_graph':
                            object_graph = json.loads(request.params.values[i])
                            obj_count = len(object_graph) if isinstance(object_graph, list) else "unknown"
                            self.get_logger().info(f'Object Graph: Found {obj_count} objects')
                            if object_graph:
                                self.get_logger().info(f'Object Graph Content:\n{json.dumps(object_graph, indent=2)}')
                            else:
                                self.get_logger().info('Object Graph: Empty')
                        elif key == 'rtdl_syntax':
                            rtdl_syntax = json.loads(request.params.values[i])
                            self.get_logger().info('RTDL Syntax: Received')
                        elif key == 'skill_primitive_specs':
                            skill_primitive_specs = json.loads(request.params.values[i])
                            self.get_logger().info('Skill/Primitive Specs: Received')
                        elif key == 'skill_specs':
                            skill_specs = json.loads(request.params.values[i])
                            skill_count = len(skill_specs) if isinstance(skill_specs, list) else "unknown"
                            self.get_logger().info(f'Skill Specs: Found {skill_count} skills')
                            if skill_specs:
                                self.get_logger().info(f'Available Skills:\n{json.dumps(skill_specs, indent=2)}')
                    except Exception as e:
                        self.get_logger().warn(f'Failed to parse {key} from params: {e}')
        
        # Generate RTDL code
        self.get_logger().info('-' * 80)
        self.get_logger().info('Generating RTDL code...')
        rtdl_code = self._generate_rtdl(request.description, object_graph, rtdl_syntax, skill_primitive_specs, skill_specs)
        
        response.rtdl = rtdl_code
        response.rtdl_type = 'list'  # Simple list-style RTDL
        response.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info('-' * 80)
        self.get_logger().info(f'Generated RTDL (type: {response.rtdl_type}, length: {len(rtdl_code)} chars)')
        self.get_logger().info(f'RTDL Code:\n{rtdl_code}')
        self.get_logger().info('=' * 80)
        return response

    def _generate_rtdl(self, description, object_graph=None, rtdl_syntax=None, skill_primitive_specs=None, skill_specs=None):
        """
        Generate simple list-style RTDL code from description using DeepSeek LLM.
        Format: JSON array of instructions, each with type, name, and params.
        """
        if not self.deepseek_client:
            raise RuntimeError('DeepSeek client not initialized')
        
        return self._generate_rtdl_with_deepseek(description, object_graph, rtdl_syntax, skill_primitive_specs, skill_specs)
    
    def _generate_rtdl_with_deepseek(self, description, object_graph=None, rtdl_syntax=None, skill_primitive_specs=None, skill_specs=None):
        """Generate RTDL using DeepSeek LLM API."""
        self.get_logger().info('Using DeepSeek LLM for task planning')
        
        # Build system prompt with RTDL syntax if provided
        if rtdl_syntax:
            system_prompt = f"""You are a robot task planning expert. Your task is to convert natural language task descriptions into RTDL (Robot Task Description Langauge) code.

RTDL Syntax Specification:
{json.dumps(rtdl_syntax, indent=2)}

Important:
- Only output valid JSON, no additional text
- Follow the RTDL syntax specification exactly
- ALWAYS include "object_id" field in each instruction (required to specify which object executes the instruction)
- Use object_id from the object graph (usually the robot object)
- Use skills/primitives available in the specified object's registered_skills/registered_primitives
- When task description mentions an object, find the best matching object from object_graph by label
- Use the object's 'id' (UUID) to reference it in params - this is the unique identifier
- Labels may not exactly match (e.g., 'plant' vs 'potted plant') - use semantic understanding to find the best match
- Keep instructions simple and sequential"""
        else:
            # Default RTDL format
            system_prompt = """You are a robot task planning expert. Your task is to convert natural language task descriptions into RTDL (Robot Task Description Langauge) code.

RTDL Format:
- Output a JSON array of instructions
- Each instruction is a JSON object with:
  - "object_id": string (REQUIRED) - ID of the object that executes this instruction (usually robot, e.g., "robot_001")
  - "type": "skill" or "primitive"
  - "name": skill/primitive name (e.g., "skl::pick", "skl::place", "skl::navigate", "prm::arm.move.ee") # remember to prefix with "skl::" or "prm::"
  - "params": JSON object with skill/primitive parameters

Example:
[
  {"object_id": "robot_001", "type": "skill", "name": "skl::pick", "params": {"target": "cup_001"}},
  {"object_id": "robot_001", "type": "skill", "name": "skl::place", "params": {"target": "cup_001", "destination": "table_001"}}
]

Important:
- Only output valid JSON, no additional text
- ALWAYS include "object_id" field in each instruction (required)
- Use object_id from the object graph (usually the robot object)
- Use skills/primitives available in the specified object's registered_skills/registered_primitives
- When task description mentions an object, find the best matching object from object_graph by label
- Use the object's 'id' (UUID) to reference it in params - this is the unique identifier
- Labels may not exactly match (e.g., 'plant' vs 'potted plant') - use semantic understanding
- Keep instructions simple and sequential"""

        user_prompt = f"Task description: {description}\n\n"
        
        # Add object graph information
        if object_graph:
            user_prompt += f"Available objects in environment:\n{json.dumps(object_graph, indent=2)}\n\n"
            
            # CRITICAL: Object selection instructions
            user_prompt += "=" * 80 + "\n"
            user_prompt += "OBJECT SELECTION RULES:\n"
            user_prompt += "=" * 80 + "\n"
            user_prompt += "1. Each object has a unique 'id' (UUID) and a 'label' (name).\n"
            user_prompt += "2. When the task description mentions an object (e.g., 'go to the plant at corner'):\n"
            user_prompt += "   - Find the BEST MATCHING object by comparing the description with object labels\n"
            user_prompt += "   - Labels may not exactly match (e.g., task says 'plant' but object is 'potted plant')\n"
            user_prompt += "   - Use semantic understanding to find the most relevant object\n"
            user_prompt += "   - Consider location hints (e.g., 'at corner', 'on the table')\n"
            user_prompt += "3. To reference an object in RTDL params, use BOTH:\n"
            user_prompt += "   - The object's 'id' (UUID) as the unique identifier\n"
            user_prompt += "   - The object's 'label' for human readability\n"
            user_prompt += "4. Example: If task is 'go to the plant' and object graph has:\n"
            user_prompt += "   {\"id\": \"abc123\", \"label\": \"potted plant\", ...}\n"
            user_prompt += "   Then in RTDL params, use: {\"target_object_id\": \"abc123\", \"target_object_label\": \"potted plant\"}\n"
            user_prompt += "   OR if the skill accepts just an ID: {\"target\": \"abc123\"}\n"
            user_prompt += "5. The 'id' (UUID) is the UNIQUE identifier - always use it to specify which object to interact with.\n"
            user_prompt += "=" * 80 + "\n\n"
            
            # Extract robot object and its skills/primitives
            robot_objects = [obj for obj in object_graph if 'robot' in obj.get('label', '').lower() or 'robot' in obj.get('id', '').lower() or obj.get('id') == 'robot_self']
            if robot_objects:
                robot = robot_objects[0]
                robot_id = robot.get('id', 'robot_self')
                robot_skills = robot.get('registered_skills', [])
                robot_primitives = robot.get('registered_primitives', [])
                user_prompt += f"Robot object ID: {robot_id}\n"
                user_prompt += f"Robot registered skills: {robot_skills}\n"
                user_prompt += f"Robot registered primitives: {robot_primitives}\n"
                user_prompt += f"IMPORTANT: Use object_id='{robot_id}' in all RTDL instructions to specify that this robot executes them.\n"
            else:
                user_prompt += "WARNING: No robot object found in object graph. You must still include 'object_id' field in each instruction.\n"
            
            user_prompt += "\n"
            user_prompt += "When the task description mentions an object:\n"
            user_prompt += "- Search through the object graph to find the best matching object by label\n"
            user_prompt += "- Use the object's 'id' (UUID) to reference it in RTDL params\n"
            user_prompt += "- If multiple objects match, choose based on context (location, description hints)\n"
            user_prompt += "\n"
        
        # Add skill and primitive specifications
        if skill_primitive_specs:
            user_prompt += f"\nAvailable Skills and Primitives:\n{json.dumps(skill_primitive_specs, indent=2)}\n\n"
            user_prompt += "When generating RTDL, use the input/output parameter types from the specifications above.\n"
            user_prompt += "Ensure all parameters match the expected types (string, float, bool, geometry_msgs/msg/PoseStamped, etc.).\n"
        
        # Add skill specs from system API (preferred over skill_primitive_specs)
        if skill_specs:
            user_prompt += f"\nAvailable Skills (from system API):\n{json.dumps(skill_specs, indent=2)}\n\n"
            user_prompt += "=" * 80 + "\n"
            user_prompt += "SKILL USAGE INSTRUCTIONS:\n"
            user_prompt += "=" * 80 + "\n"
            user_prompt += "IMPORTANT: Use ONLY the skills listed above. Each skill has:\n"
            user_prompt += "- name: The skill name (e.g., 'skl::wandering', 'skl::move_to_object')\n"
            user_prompt += "- start_args: The input parameter schema (JSON object describing required/optional parameters)\n"
            user_prompt += "  * This schema tells you what parameters the skill expects\n"
            user_prompt += "  * Required parameters MUST be included in 'params'\n"
            user_prompt += "  * Optional parameters can be included if needed\n"
            user_prompt += "- start_topic: The topic to publish to start the skill\n"
            user_prompt += "\n"
            user_prompt += "When generating RTDL:\n"
            user_prompt += "1. Use the exact skill name from the 'name' field (e.g., 'skl::wandering', 'skl::move_to_object')\n"
            user_prompt += "2. Read the 'start_args' schema carefully to understand what parameters the skill expects\n"
            user_prompt += "3. For object-related skills (e.g., 'skl::move_to_object'):\n"
            user_prompt += "   - Use 'target_object_id' parameter with the object's UUID (from object_graph)\n"
            user_prompt += "   - The UUID is the unique identifier for the object\n"
            user_prompt += "4. Ensure all required parameters in 'params' match the types specified in 'start_args'\n"
            user_prompt += "5. Only use skills that are listed above\n"
            user_prompt += "=" * 80 + "\n\n"
        
        user_prompt += "\nGenerate RTDL code for this task:"
        
        # Log the full prompt
        self.get_logger().info('-' * 80)
        self.get_logger().info('DeepSeek LLM Prompt:')
        self.get_logger().info('System Prompt:')
        self.get_logger().info(system_prompt)
        self.get_logger().info('User Prompt:')
        self.get_logger().info(user_prompt)
        self.get_logger().info('-' * 80)
        
        # Call DeepSeek API
        import time
        start_time = time.time()
        self.get_logger().info('Calling DeepSeek API...')
        response = self.deepseek_client.chat.completions.create(
            model="deepseek-chat",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.1,
        )
        elapsed_time = time.time() - start_time
        
        llm_response = response.choices[0].message.content.strip()
        self.get_logger().info(f'DeepSeek API call completed in {elapsed_time:.2f} seconds')
        self.get_logger().info('-' * 80)
        self.get_logger().info('DeepSeek LLM Response (full):')
        self.get_logger().info(llm_response)
        self.get_logger().info('-' * 80)
        
        # Try to extract JSON from response (might have markdown code blocks)
        if llm_response.startswith('```'):
            # Remove markdown code blocks
            lines = llm_response.split('\n')
            json_start = None
            json_end = None
            for i, line in enumerate(lines):
                if line.strip().startswith('```'):
                    if json_start is None:
                        json_start = i + 1
                    else:
                        json_end = i
                        break
            if json_start and json_end:
                llm_response = '\n'.join(lines[json_start:json_end])
        
        # Parse JSON response
        try:
            instructions = json.loads(llm_response)
            if not isinstance(instructions, list):
                instructions = [instructions]
            
            # Validate and format instructions
            formatted_instructions = []
            for inst in instructions:
                if isinstance(inst, dict):
                    inst_object_id = inst.get('object_id', 'robot_001')  # Default to robot_001 if missing
                    inst_type = inst.get('type', 'skill')
                    inst_name = inst.get('name', 'unknown')
                    inst_params = inst.get('params', {})
                    formatted_instructions.append({
                        'object_id': inst_object_id,  # REQUIRED: object that executes this instruction
                        'type': inst_type,
                        'name': inst_name,
                        'params': inst_params
                    })
            
            # Convert to JSON string (RTDL format)
            return json.dumps(formatted_instructions, indent=2)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse DeepSeek response as JSON: {e}')
            self.get_logger().error(f'Response was: {llm_response}')
            raise RuntimeError(f'Failed to parse DeepSeek response as JSON: {e}')
    
    def _validate_api_key(self, api_key: str) -> bool:
        """
        Validate DeepSeek API key format.
        DeepSeek API keys typically start with 'sk-' and are at least 32 characters long.
        """
        if not api_key or not isinstance(api_key, str):
            return False
        
        # Remove whitespace
        api_key = api_key.strip()
        
        # Check minimum length (DeepSeek API keys are typically 32+ characters, including 'sk-' prefix)
        # Minimum: 'sk-' (3) + key part (32) = 35 characters
        if len(api_key) < 35:
            return False
        
        # Check if starts with 'sk-' (common format for API keys)
        if not api_key.startswith('sk-'):
            return False
        
        # Check if contains only valid characters (alphanumeric, hyphens, underscores)
        # After 'sk-' prefix
        key_part = api_key[3:]
        if not key_part.replace('-', '').replace('_', '').isalnum():
            return False
        
        # Check that key part has reasonable length (at least 32 characters)
        if len(key_part) < 32:
            return False
        
        return True


def main(args=None):
    rclpy.init(args=args)
    task_plan_service = TaskPlanService()
    rclpy.spin(task_plan_service)
    task_plan_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

