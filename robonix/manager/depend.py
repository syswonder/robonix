import os
import yaml
from collections import defaultdict
from constant import BASE_PATH
from log import logger
from node import get_entry_name


def scan_dir(base_path, feature_set, feature_sources, all_dependencies, all_params):
    """Scan directory for description.yml files and extract features, dependencies, and params"""
    if not os.path.exists(base_path):
        logger.warning(f"Path {base_path} does not exist, skipping")
        return

    for root, dirs, files in os.walk(base_path):
        if "description.yml" in files:
            desc_path = os.path.join(root, "description.yml")
            logger.debug(f"Processing description file: {desc_path}")
            try:
                with open(desc_path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f)
                    features = data.get("feature", [])
                    dependencies = data.get("depend", [])
                    params = data.get("params", None)
                    if params:
                        all_params[base_path] = params

                    if not features:
                        logger.warning(
                            f"No 'feature' key found in {desc_path}, skipping"
                        )
                        continue

                    for feature in features:
                        if feature in feature_set:
                            raise ValueError(
                                f"Duplicate feature '{feature}' found in:\n"
                                f"  - {feature_sources[feature]}\n"
                                f"  - {desc_path}"
                            )

                        feature_set.add(feature)
                        feature_sources[feature] = desc_path
                    all_dependencies[base_path] = dependencies

            except Exception as e:
                logger.error(f"Failed to read {desc_path}: {e}")


def check_depend(config_path):
    """
    Scan all subdirectories in cap/ and skill/ for description.yml files,
    collect features and dependencies.
    - Check for duplicate features
    - Check if dependencies exist in feature list
    """
    feature_set = set()
    feature_sources = {}  # feature -> file path
    all_dependencies = defaultdict(list)  # feature -> list of dependencies
    all_params = {}  # feature -> params dict
    all_valid = True

    config = {}
    config_path = os.path.join(BASE_PATH, config_path)
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    logger.info(f"Starting dependency check for config: {config_path}")

    # Step 1: Scan all directories to get features and dependencies
    for base, entries in config.items():
        if entries is None:
            logger.warning(f"No entries found in {base}")
            continue

        base_dir_path = os.path.join(BASE_PATH, base)
        if not os.path.exists(base_dir_path):
            logger.error(f"Base directory not found at '{base_dir_path}'")
            return []
        if not os.path.isdir(base_dir_path):
            logger.error(f"'{base_dir_path}' exists but is not a directory")
            return []

        try:
            logger.debug(f"Scanning base directory: {base_dir_path}")
            for entry in entries:
                entry_name, entry_content = get_entry_name(entry)
                sub_dir_path = os.path.join(base_dir_path, entry_name)
                scan_dir(sub_dir_path, feature_set, feature_sources, all_dependencies, all_params)
                if sub_dir_path in all_params:
                    for param_name, param_attr in all_params[sub_dir_path].items():
                        if "required" in param_attr and param_attr["required"]:
                            logger.info(f"param required: ",entry[entry_name].get('params',{}))
                            # there is no param or param not defined
                            if "params" not in entry[entry_name] or param_name not in entry[entry_name]["params"]:
                                # use default value if exists
                                if "default" in param_attr:
                                    entry[entry_name]["params"] = entry[entry_name].get('params',{})
                                    entry[entry_name]["params"][param_name] = param_attr["default"]
                                else:
                                    logger.error(f"node '{entry_name}' requires param '{param_name}' but it is not defined.")
                                    all_valid = False
                        # not required
                        else:
                            if "default" in param_attr:
                                entry[entry_name]["params"] = entry[entry_name].get('params',{})
                                entry[entry_name]["params"][param_name] = param_attr["default"]
                            else:
                                logger.error(f"node '{entry_name}' param '{param_name}' is not required and has no default value.")
                                all_valid = False
        except Exception as e:
            raise ValueError(f"[ERROR] An error occurred while accessing '{base_dir_path}': {e}")
    if not all_valid:
        raise ValueError("[FAILED] Some node params are required but not defined, or has neither required nor default value.")

    # Step 2: Check if all dependencies exist in feature_set
    all_valid = True
    for entry_path, deps in all_dependencies.items():
        for dep in deps:
            if dep not in feature_set:
                logger.error(f"Node '{entry_path}' depends on unknown feature '{dep}'")
                all_valid = False

    if not all_valid:
        raise ValueError("Some dependencies refer to unknown features")

    logger.info("All features and dependencies validated successfully")
    return feature_set, all_dependencies


# Example usage
if __name__ == "__main__":
    config_path = os.path.join(BASE_PATH, "config", "include/ranger_test.yml")
    features, dependencies = check_depend(config_path)
    logger.info(f"Features: {features}")
    logger.info(f"Dependencies: {dependencies}")
