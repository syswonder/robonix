import os
import yaml
import ast
import importlib.util
from pathlib import Path
import re
from constant import BASE_PATH, BASE_CAP_PATH
from log import logger

import sys

if BASE_CAP_PATH not in sys.path:
    sys.path.append(BASE_CAP_PATH)


def configure_api_module(dir_path):
    """
    Configure API module, automatically generate __init__.py and set up imports

    :param dir_path: The specified directory path
    """
    dir_path = Path(dir_path).resolve()

    # 1. Read name field from description.yml
    description_path = dir_path / "description.yml"
    if not description_path.exists():
        raise FileNotFoundError(f"description.yml not found at {description_path}")

    with open(description_path, "r", encoding="utf-8") as f:
        description_data = yaml.safe_load(f)

    name = description_data.get("name")
    if not name:
        raise ValueError("'name' field not found in description.yml")

    # 2. Read plugins from ROOT_DIR/config/plugin.yml
    root_dir = dir_path.parent.parent  # assume ROOT_DIR is the grandparent of dir_path
    plugins_config_path = root_dir / "config" / "plugins.yml"

    if not plugins_config_path.exists():
        raise FileNotFoundError(f"plugin.yml not found at {plugins_config_path}")

    with open(plugins_config_path, "r", encoding="utf-8") as f:
        plugins_data = yaml.safe_load(f)

    plugin = plugins_data.get(name)
    if not plugin:
        raise ValueError(f"No plugin found for '{name}' in plugin.yml")

    logger.info(f"Configuring API for '{name}' with plugin: {plugin}")

    # 3. Find functions decorated with @eaios.api in api.py
    api_path = dir_path / "api" / "api.py"
    if not api_path.exists():
        raise FileNotFoundError(f"api.py not found at {api_path}")

    # Parse api.py to find decorated functions
    with open(api_path, "r", encoding="utf-8") as f:
        api_code = f.read()

    # Use AST parsing to find decorated functions
    decorated_functions = []
    tree = ast.parse(api_code)

    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            for decorator in node.decorator_list:
                # Check if decorator is @eaios.api
                if (
                    isinstance(decorator, ast.Attribute)
                    and decorator.attr == "api"
                    and isinstance(decorator.value, ast.Name)
                    and decorator.value.id == "eaios"
                ):
                    decorated_functions.append(node.name)

    if not decorated_functions:
        logger.warning("No functions decorated with @eaios.api found in api.py")

    logger.debug(f"Decorated functions: {decorated_functions}")

    # 4. Search for functions with same names in plugin directory
    plugins_dir = dir_path / "plugins"
    imports = {}

    plugin_path = plugins_dir / plugin

    if not plugin_path.exists():
        logger.warning(f"Plugin directory not found: {plugin_path}, skipping")
    else:

        # Iterate through all Python files in plugin directory
        for file_path in plugin_path.rglob("*.py"):
            if file_path.name == "__init__.py":
                continue

            module_name = file_path.stem
            rel_path = file_path.relative_to(plugin_path).with_suffix("")
            module_path = ".".join(rel_path.parts)

            # dynamically import module
            spec = importlib.util.spec_from_file_location(module_name, file_path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)

            # Check if there are functions with same names as decorated functions
            for func in decorated_functions:
                if hasattr(module, func) and callable(getattr(module, func)):
                    # Record import path
                    import_path = f"{dir_path.name}.plugin.{plugin}.{module_path}"
                    imports.setdefault(func, []).append(import_path)

    # 5. Generate __init__.py content
    init_content = "# AUTO-GENERATED FILE - DO NOT EDIT MANUALLY\n\n"
    init_content += "# Imports for decorated API functions\n"

    # Generate import statements for each function
    for func, paths in imports.items():
        if len(paths) > 1:
            logger.warning(
                f"Multiple implementations found for {func}, using first one"
            )

        import_path = paths[0]
        init_content += f"from {import_path} import {func}\n"

    # add import of original API
    init_content += "\n# Import the original API module\n"
    init_content += f"from .api import *\n"

    # 6. Write to __init__.py file
    init_path = dir_path / "api" / "__init__.py"
    with open(init_path, "w", encoding="utf-8") as f:
        f.write(init_content)

    logger.info(f"Generated {init_path} with {len(imports)} imports")

    # 7. Tell user what needs to be done in api.py file
    logger.info("\nTo use the imported functions in api.py:")
    logger.info("1. Add the following import at the TOP of your api.py file:")
    logger.info("   from . import function_name  # for each function you need")
    logger.info("2. You can then use the functions directly in your code")

    return list(imports.keys())


# Usage example
if __name__ == "__main__":
    # specify directory path
    target_dir = os.path.join(BASE_CAP_PATH, "navigation2")

    # configure API module
    imported_functions = configure_api_module(target_dir)

    logger.info(f"Imported functions: {imported_functions}")
