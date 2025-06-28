import os
import yaml # 导入 pyyaml 库

class BaseNode:
    def __init__(self, cwd: str, name: str, version: str, author: str, startup_on_boot: bool, startup_command: str):
        """
        Args:
            cwd (str): The name of the base directory.
            name (str): The name of the base.
            version (str): The version of the base.
            author (str): The author of the base.
            startup_on_boot (bool): Whether the base should start on boot.
            startup_command (str): The command to start the base.
        """
        self.cwd = cwd
        self.name = name
        self.version = version
        self.author = author
        self.start_on_boot = startup_on_boot
        self.startup_command = startup_command

    def __str__(self):
        return f"BaseNode(cwd='{self.cwd}', name='{self.name}', version='{self.version}', author='{self.author}', start_on_boot='{self.start_on_boot}', startup_command='{self.startup_command}')"
    
    def __repr__(self):
        return f"BaseNode({self.name}_{self.version}, cwd='{self.cwd}')"


def get_node_details(target_path: str) -> list[BaseNode]:
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
    base_dir_path = target_path
    all_base_details = []

    if not os.path.exists(base_dir_path):
        print(f"Error: The 'base' directory was not found at '{base_dir_path}'")
        return []
    if not os.path.isdir(base_dir_path):
        print(f"Error: '{base_dir_path}' exists but is not a directory.")
        return []

    try:
        # List all entries in the 'base' directory
        all_entries = os.listdir(base_dir_path)

        for entry in all_entries:
            sub_dir_path = os.path.join(base_dir_path, entry)
            print(f"Checking: {entry}")

            # Check if the entry is a directory
            if os.path.isdir(sub_dir_path):
                description_file_path = os.path.join(sub_dir_path, "description.yml")

                # Check if description.yml exists in the subdirectory
                if os.path.exists(description_file_path) and os.path.isfile(description_file_path):
                    print(f"Found description.yml in: {sub_dir_path}")
                    try:
                        # Read and parse the YAML file
                        with open(description_file_path, 'r', encoding='utf-8') as f:
                            data = yaml.safe_load(f)

                            base_info = BaseNode(
                                cwd=sub_dir_path,
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
        print(f"An error occurred while accessing '{base_dir_path}': {e}")
        return []

    return all_base_details

if __name__ == "__main__":
    import sys
    project_root_path = sys.argv[1]
    all_base_details = get_node_details(project_root_path)
    print(all_base_details)

