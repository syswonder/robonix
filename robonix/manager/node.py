import os
import yaml
from log import logger
from constant import BASE_PATH


class BaseNode:
    def __init__(
        self,
        cwd: str,
        name: str,
        version: str,
        author: str,
        startup_on_boot: bool,
        startup_command: str,
        node_type: str = None,
        params: dict = None,
    ):
        """
        Args:
            cwd (str): The name of the base directory.
            name (str): The name of the base.
            version (str): The version of the base.
            author (str): The author of the base.
            startup_on_boot (bool): Whether the base should start on boot.
            startup_command (str): The command to start the base.
            node_type (str): The type of the node (driver, capability, etc.).
        """
        self.cwd = cwd
        self.name = name
        self.version = version
        self.author = author
        self.start_on_boot = startup_on_boot
        self.startup_command = startup_command
        self.node_type = node_type
        self.params = params

    def __str__(self):
        return f"BaseNode(cwd='{self.cwd}', name='{self.name}', version='{self.version}', author='{self.author}', start_on_boot='{self.start_on_boot}', startup_command='{self.startup_command}', params = '{self.params}')"
    
    def __repr__(self):
        return f"BaseNode({self.name}_{self.version}, cwd='{self.cwd}', params={self.params})"

def get_entry_name(entry):
    if isinstance(entry,dict):
        entry_name = list(entry.keys())[0]
        entry_content = entry[entry_name]
    else:
        entry_name = entry
        entry_content = None

    return entry_name, entry_content

def get_node(entry, sub_dir_path, node_type: str = None, entry_content: dict = None) -> BaseNode:
    """
    Helper function to create a BaseNode object from a directory entry and its description.yml file.

    Args:
        entry (str): The name of the directory entry.
        sub_dir_path (str): The path to the subdirectory containing the description.yml file.
        node_type (str): The type of the node (driver, capability, etc.).

    Returns:
        BaseNode: An instance of BaseNode with details extracted from the description.yml file.
    """
    logger.debug(f"Checking entry: {entry} content {entry_content} in path: {sub_dir_path} for description.yml")
    # Check if the entry is a directory
    if os.path.isdir(sub_dir_path):
        description_file_path = os.path.join(sub_dir_path, "description.yml")

        # Check if description.yml exists in the subdirectory
        if os.path.exists(description_file_path) and os.path.isfile(
            description_file_path
        ):
            logger.info(f"Found description.yml in: {sub_dir_path}")
            try:
                # Read and parse the YAML file
                with open(description_file_path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f)
                    params = {}
                    for k,v in data.get("params", {}).items():
                        if "default" in v:
                            params[k] = v.get("default")
                    params.update(entry_content.get("params", {}) if isinstance(entry_content,dict) else {})
                    base_info = BaseNode(
                        cwd=sub_dir_path,
                        name=data.get("name"),
                        version=data.get("version"),
                        author=data.get("author"),
                        startup_on_boot=data.get("start_on_boot", False),
                        startup_command=data.get("startup_command", None),
                        node_type=node_type,
                        params=params
                    )
                    return base_info
            except yaml.YAMLError as e:
                logger.error(f"Error parsing YAML file '{description_file_path}': {e}")
            except Exception as e:
                logger.error(
                    f"An unexpected error occurred while processing '{description_file_path}': {e}"
                )
        else:
            logger.warning(
                f"No description.yml found in '{entry}' or it's not a file. Skipping."
            )
    else:
        logger.warning(f"Skipping non-directory entry: {entry}")
    return None


def get_node_details(config_path: str) -> list[BaseNode]:
    """
    Retrieves details (name, version, author, startup_command) from description.yml
    for each direct subdirectory within the 'base' folder.

    Args:
        project_root_path (str): The absolute or relative path to the project's root directory.

    Returns:
        list: A list of BaseNode objects. Each object represents a base package
              and contains its name, version, author, and startup_command as extracted
              from its description.yml. Returns an empty list if no 'base' directory
              or no valid description.yml files are found.
    """
    config_path = os.path.join(BASE_PATH, config_path)
    if not os.path.exists(config_path):
        logger.error(f"Error: The configuration file '{config_path}' does not exist.")
        return []
    config = {}

    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)
    all_base_details = []

    for base, entries in config.items():
        if entries is None:
            logger.error(f"No entries found in {base}")
            continue
        base_dir_path = os.path.join(BASE_PATH, base)
        if not os.path.exists(base_dir_path):
            logger.error(
                f"Error: The 'base' directory was not found at '{base_dir_path}'"
            )
            return []
        if not os.path.isdir(base_dir_path):
            logger.error(f"Error: '{base_dir_path}' exists but is not a directory.")
            return []

        try:
            # List all entries in the 'base' directory

            for entry in entries:
                entry_name, entry_content = get_entry_name(entry)
                sub_dir_path = os.path.join(base_dir_path, entry_name)
                logger.info(f"Checking: {entry_name}")
                base_info = get_node(entry_name, sub_dir_path, base, entry_content)
                if base_info:
                    all_base_details.append(base_info)
                else:
                    logger.warning(f"No valid BaseNode found for entry_name: {entry_name}")
        except Exception as e:
            logger.error(f"An error occurred while accessing '{base_dir_path}': {e}")
            return []

    return all_base_details


if __name__ == "__main__":
    import sys

    project_root_path = sys.argv[1]
    all_base_details = get_node_details(project_root_path)
    logger.info(all_base_details)
