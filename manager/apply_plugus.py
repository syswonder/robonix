import os
import yaml
import ast
import importlib.util
from pathlib import Path
import re
from constant import BASE_PATH,BASE_CAP_PATH

import sys
if BASE_CAP_PATH not in sys.path:
    sys.path.append(BASE_CAP_PATH)

def configure_api_module(dir_path):
    """
    配置API模块，自动生成__init__.py并设置导入
    
    :param dir_path: 指定的目录路径
    """
    # 确保路径是Path对象
    dir_path = Path(dir_path).resolve()
    
    # 1. 从description.yml读取name字段
    description_path = dir_path / "description.yml"
    if not description_path.exists():
        raise FileNotFoundError(f"description.yml not found at {description_path}")
    
    with open(description_path, 'r', encoding='utf-8') as f:
        description_data = yaml.safe_load(f)
    
    name = description_data.get('name')
    if not name:
        raise ValueError("'name' field not found in description.yml")
    
    # 2. 从ROOT_DIR/config/plugin.yml读取plugins
    root_dir = dir_path.parent.parent  # 假设ROOT_DIR是dir_path的祖父目录
    plugins_config_path = root_dir / "config" / "plugins.yml"
    
    if not plugins_config_path.exists():
        raise FileNotFoundError(f"plugin.yml not found at {plugins_config_path}")
    
    with open(plugins_config_path, 'r', encoding='utf-8') as f:
        plugins_data = yaml.safe_load(f)
    
    plugin = plugins_data.get(name)
    if not plugin:
        raise ValueError(f"No plugin found for '{name}' in plugin.yml")
    
    print(f"Configuring API for '{name}' with plugin: {plugin}")
    
    # 3. 找到api.py中被@eaios.api修饰的函数
    api_path = dir_path / "api" / "api.py"
    if not api_path.exists():
        raise FileNotFoundError(f"api.py not found at {api_path}")
    
    # 解析api.py，找到被装饰的函数
    with open(api_path, 'r', encoding='utf-8') as f:
        api_code = f.read()
    
    # 使用AST解析找到被装饰的函数
    decorated_functions = []
    tree = ast.parse(api_code)
    
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            for decorator in node.decorator_list:
                # 检查装饰器是否是@eaios.api
                if (isinstance(decorator, ast.Attribute) and 
                    decorator.attr == 'api' and 
                    isinstance(decorator.value, ast.Name) and 
                    decorator.value.id == 'eaios'):
                    decorated_functions.append(node.name)
    
    if not decorated_functions:
        print("Warning: No functions decorated with @eaios.api found in api.py")

    print("decorated_functions",decorated_functions)
    
    # 4. 在插件目录中查找同名函数
    plugins_dir = dir_path / "plugins"
    imports = {}

    plugin_path = plugins_dir / plugin

    if not plugin_path.exists():
        print(f"Warning: Plugin directory not found: {plugin_path}, skipping")
    else:

        # 遍历插件目录中的所有Python文件
        for file_path in plugin_path.rglob("*.py"):
            if file_path.name == "__init__.py":
                continue

            module_name = file_path.stem
            rel_path = file_path.relative_to(plugin_path).with_suffix('')
            module_path = ".".join(rel_path.parts)

            # 动态导入模块
            spec = importlib.util.spec_from_file_location(module_name, file_path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)

            # 检查是否有与装饰函数同名的函数
            for func in decorated_functions:
                if hasattr(module, func) and callable(getattr(module, func)):
                    # 记录导入路径
                    import_path = f"{dir_path.name}.plugin.{plugin}.{module_path}"
                    imports.setdefault(func, []).append(import_path)
    
    # 5. 生成__init__.py内容
    init_content = "# AUTO-GENERATED FILE - DO NOT EDIT MANUALLY\n\n"
    init_content += "# Imports for decorated API functions\n"
    
    # 为每个函数生成导入语句
    for func, paths in imports.items():
        if len(paths) > 1:
            print(f"Warning: Multiple implementations found for {func}, using first one")
        
        import_path = paths[0]
        init_content += f"from {import_path} import {func}\n"
    
    # 添加原始API的导入
    init_content += "\n# Import the original API module\n"
    init_content += f"from .api import *\n"
    
    # 6. 写入__init__.py
    init_path = dir_path / "api" / "__init__.py"
    with open(init_path, 'w', encoding='utf-8') as f:
        f.write(init_content)
    
    print(f"Generated {init_path} with {len(imports)} imports")
    
    # 7. 告诉用户在api.py中需要做什么
    print("\nTo use the imported functions in api.py:")
    print("1. Add the following import at the TOP of your api.py file:")
    print("   from . import function_name  # for each function you need")
    print("2. You can then use the functions directly in your code")
    
    return list(imports.keys())

# 使用示例
if __name__ == "__main__":
    # 指定目录路径
    target_dir = os.path.join(BASE_CAP_PATH,"navigation2")
    
    # 配置API模块
    imported_functions = configure_api_module(target_dir)
    
    print(f"\nImported functions: {imported_functions}")