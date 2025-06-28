import os
from node import BaseNode
import yaml # 导入 pyyaml 库

class CapabilityNode(BaseNode):
    def __init__(self, directory_path: str, name: str, version: str, author: str, startup_on_boot: bool, startup_command: str):
        """
        Args:
            directory_path (str): The name of the capability directory.
            name (str): The name of the capability.
            version (str): The version of the capability.
            author (str): The author of the capability.
            startup_on_boot (bool): Whether the capability should start on boot.
            startup_command (str): The command to start the capability.
        """
        super().__init__(directory_path, name, version, author, startup_on_boot, startup_command)

    def __str__(self):
        return f"CapabilityNode(cwd='{self.cwd}', name='{self.name}', version='{self.version}', author='{self.author}', start_on_boot='{self.start_on_boot}', startup_command='{self.startup_command}')"
    
    def __repr__(self):
        return f"CapabilityNode({self.name}_{self.version}, cwd='{self.cwd}')"


def get_capability_details(project_root_path: str) -> list[CapabilityNode]:
    """
    Retrieves details (name, version, author, startup_command) from description.yml
    for each direct subdirectory within the 'capability' folder.

    Args:
        project_root_path (str): The absolute or relative path to the project's root directory.

    Returns:
        list: A list of CapabilityInfo objects. Each object represents a capability package
              and contains its name, version, author, and startup_command as extracted
              from its description.yml. Returns an empty list if no 'capability' directory
              or no valid description.yml files are found.
    """
    capability_dir_path = os.path.join(project_root_path, "capability")
    all_base_details = []

    if not os.path.exists(capability_dir_path):
        print(f"Error: The 'base' directory was not found at '{capability_dir_path}'")
        return []
    if not os.path.isdir(capability_dir_path):
        print(f"Error: '{capability_dir_path}' exists but is not a directory.")
        return []

    try:
        # List all entries in the 'base' directory
        all_entries = os.listdir(capability_dir_path)

        for entry in all_entries:
            node_dir_path = os.path.join(capability_dir_path, entry)

            # Check if the entry is a directory
            if os.path.isdir(node_dir_path):
                description_file_path = os.path.join(node_dir_path, "description.yml")

                # Check if description.yml exists in the subdirectory
                if os.path.exists(description_file_path) and os.path.isfile(description_file_path):
                    print(f"Found description.yml in: {entry}")
                    try:
                        # Read and parse the YAML file
                        with open(description_file_path, 'r', encoding='utf-8') as f:
                            data = yaml.safe_load(f)

                            base_info = CapabilityNode(
                                directory_path=node_dir_path,
                                name=data.get("name"),
                                version=data.get("version"),
                                author=data.get("author"),
                                startup_on_boot=data.get("start_on_boot", False),
                                startup_command=data.get("startup_command")
                            )
                            all_base_details.append(base_info)

                    except yaml.YAMLError as e:
                        print(f"Error parsing YAML file '{description_file_path}': {e}")
                    except Exception as e:
                        print(f"An unexpected error occurred while processing '{description_file_path}': {e}")
                else:
                    print(f"Warning: No description.yml found in '{entry}' or it's not a file. Skipping.")
            else:
                print(f"Skipping non-directory entry: {entry}") # 可以打印非目录项

    except Exception as e:
        print(f"An error occurred while accessing '{capability_dir_path}': {e}")
        return []

    return all_base_details

if __name__ == "__main__":
    project_root_path = "./"
    all_capability_details = get_capability_details(project_root_path)
    print(all_capability_details)

