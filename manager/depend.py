import os
import yaml
from collections import defaultdict
from constant import BASE_PATH

def scan_dir(base_path, feature_set, feature_sources, all_dependencies):
    if not os.path.exists(base_path):
        print(f"[WARN] {base_path} not existe, skipping.")
    for root, dirs, files in os.walk(base_path):
        if "description.yml" in files:
            desc_path = os.path.join(root, "description.yml")
            print(desc_path)
            try:
                with open(desc_path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f)
                    features = data.get("feature",[])
                    dependencies = data.get("depend", [])

                    if not features:
                        print(f"[WARN] {desc_path} has no 'feature' key, skipping.")
                        continue
                    for feature in features:
                        if feature in feature_set:
                            raise ValueError(f"[ERROR] Duplicate feature '{feature}' found in:\n"
                                                f"  - {feature_sources[feature]}\n"
                                                f"  - {desc_path}")

                        feature_set.add(feature)
                        feature_sources[feature] = desc_path
                    all_dependencies[base_path] = dependencies

            except Exception as e:
                print(f"[ERROR] Failed to read {desc_path}: {e}")

def check_depend(config_path):
    """
    遍历 cap/ 和 skill/ 所有子目录中的 description.yml 文件，收集 feature 和 dependency。
    - 检查重复 feature
    - 检查 dependency 是否在 feature 列表中
    """

    feature_set = set()
    feature_sources = {}  # feature -> file path
    all_dependencies = defaultdict(list)  # feature -> list of dependencies


    config = {}
    config_path = os.path.join(BASE_PATH, config_path)
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    all_base_details = []
    # Step 1: 扫描所有目录，获取 feature 和 dependency
    for base,entrys in config.items():
        if entrys is None:
            print(f"No entry find in {base}")
            continue
        base_dir_path = os.path.join(BASE_PATH, base)
        if not os.path.exists(base_dir_path):
            print(f"[ERROR] The 'base' directory was not found at '{base_dir_path}'")
            return []
        if not os.path.isdir(base_dir_path):
            print(f"[ERROR] '{base_dir_path}' exists but is not a directory.")
            return []

        try:
            # List all entries in the 'base' directory
            for entry in entrys:
                sub_dir_path = os.path.join(base_dir_path, entry)
                scan_dir(sub_dir_path, feature_set, feature_sources, all_dependencies)
        except Exception as e:
            print(f"[ERROR] An error occurred while accessing '{base_dir_path}': {e}")
            return []
    # Step 2: 检查所有依赖是否存在于 feature_set 中
    all_valid = True
    for entry_path, deps in all_dependencies.items():
        for dep in deps:
            if dep not in feature_set:
                print(f"[ERROR] node '{entry_path}' depends on unknown feature '{dep}'")
                all_valid = False

    if not all_valid:
        raise ValueError("[FAILED] Some dependencies refer to unknown features.")

    print("[SUCCESS] All features and dependencies validated.")
    return feature_set, all_dependencies

# Example usage
if __name__ == "__main__":

    config_path = os.path.join(BASE_PATH, "config", "include/ranger_test.yml")
    features, dependencies = check_depend(config_path)
    print(f"Features: {features}")
    print(f"Dependencies: {dependencies}")
