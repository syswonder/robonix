import os
import importlib.util
import yaml

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))



def import_from_first_available(yaml_path,target_globals=None):
    load_dict = {}
    """
    从YAML文件中读取路径列表，导入第一个存在的Python模块
    
    :param yaml_path: 包含文件路径列表的YAML文件路径
    """
    # 读取YAML文件
    with open(yaml_path, 'r') as f:
        file_deps = yaml.safe_load(f)
    
    for dep in file_deps.keys():
        file_paths_dict = file_deps[dep]
    
        # 确保读取到的是列表
        if not isinstance(file_paths_dict, dict):
            raise ValueError("YAML文件内容必须是路径列表")
        
        # 遍历所有文件路径
        for file_name in file_paths_dict.keys():
            if dep in load_dict.keys():
                break
            file_path = os.path.join(SCRIPT_DIR,file_name + ".py")
            # 检查文件是否存在
            if not os.path.exists(file_path):
                print(f"文件不存在: {file_path}")
                continue
                
            # 检查是否为Python文件
            if not file_path.endswith('.py'):
                print(f"跳过非Python文件: {file_path}")
                continue
                
            # 动态导入模块
            module_name = os.path.splitext(os.path.basename(file_path))[0]
            # spec = importlib.util.spec_from_file_location(module_name, file_path)
            # module = importlib.util.module_from_spec(spec)
            # print(module, spec, module_name)
            module = importlib.import_module(module_name)
            
            try:
                print("准备更新全局命名空间...")
                print("当前全局变量:", globals().keys())
                print("模块中的变量:", vars(module).keys())
                import_names = {}
                less_names=  []
                for name_to_import in file_paths_dict[file_name]:
                    if name_to_import not in vars(module).keys():
                        less_names.append(name_to_import)
                        continue
                    import_names.update({name_to_import: vars(module)[file_paths_dict[file_name]]})
                if len(import_names) != len(file_paths_dict[file_name]):
                    print(f"{file_name} 模块不全，缺少 {less_names}")
                    continue
                globals().update(import_names)
                if target_globals is not None:
                    target_globals.update(import_names)
                load_dict[dep] = [file_name,import_names]
                print("更新全局命名空间完成")
            except Exception as e:
                print(f"导入失败 {file_path}: {str(e)}")
                continue
        
        # 所有文件都不可用
        if dep not in load_dict.keys():
            raise ImportError("所有文件均不可导入")
    return load_dict

# 使用示例
if __name__ == "__main__":
    try:
        load_dict = import_from_first_available("dep.yml")
        print(globals().keys())
        print(load_dict)
        # 此处可以访问导入模块的内容
        # 例如：如果导入的模块中有函数hello(), 可以直接调用 hello()
        load_dict["hello"][2]()
    except Exception as e:
        print(f"错误: {str(e)}")