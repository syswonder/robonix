from capability import *
import os
import yaml
import subprocess
import threading
import pty
import select

# CapabilityProcess class
class CapabilityProcess:
    """
    Encapsulates a running subprocess and manages its input/output.
    """
    def __init__(self, capability_node: CapabilityNode, project_root: str, max_output: int = 2000):
        self.capability_node = capability_node
        self.project_root = project_root
        self.process = None  # Subprocess Popen object
        self.max_output = max_output
        self.master_fd, self.slave_fd = pty.openpty()
        self._running = False

    def start(self):
        """Start the process associated with the capability."""
        if self.process and self.process.poll() is None:
            print(f"Capability '{self.capability_node.name}' is already running.")
            return True

        command = self.capability_node.startup_command
        if not command:
            print(f"Error: No startup command defined for '{self.capability_node.name}'.")
            return False

        # Construct the process working directory
        process_cwd = self.capability_node.cwd
        if not os.path.exists(process_cwd):
            print(f"Error: Process working directory '{process_cwd}' does not exist for '{self.capability_node.name}'.")
            return False

        try:
            # shell=True allows running shell commands with pipes or compound commands, but has security risks
            # Prefer shell=False and split the command into a list if possible
            # Here we assume startup_command is safe and needs to be run in a shell
            self.process = subprocess.Popen(
                "stdbuf -oL " + command,
                shell=True,
                cwd=process_cwd,  # Run in the directory specific to this capability
                stdin=self.slave_fd,
                stdout=self.slave_fd,
                stderr=subprocess.STDOUT,
                text=True,  # Ensure text mode input/output (Python 3.6+)
                bufsize=1,  # Line buffering
                universal_newlines=True  # Legacy compatibility, same as text=True
            )
            
            self._running = True
            print(f"Started capability '{self.capability_node.name}' with PID: {self.process.pid} from '{process_cwd}'.")
            return True
        except FileNotFoundError:
            print(f"Error: Command '{command.split()[0]}' not found for '{self.capability_node.name}'.")
            return False
        except Exception as e:
            print(f"Error starting capability '{self.capability_node.name}': {e}")
            return False

    def stop(self, timeout=5):
        """Stop the process associated with the capability."""
        if self.process and self.process.poll() is None:  # poll() is None means the process is still running
            print(f"Stopping capability '{self.capability_node.name}' (PID: {self.process.pid})...")
            try:
                self._running = False  # Notify the reading thread to exit
                self.process.terminate()  # Send SIGTERM
                self.process.wait(timeout=timeout)  # Wait for graceful shutdown

                if self.process.poll() is None:  # If the process hasn't terminated
                    print(f"Capability '{self.capability_node.name}' did not terminate gracefully, killing it.")
                    self.process.kill()  # Send SIGKILL to forcefully terminate
                    self.process.wait()  # Ensure the process is completely terminated
                print(f"Capability '{self.capability_node.name}' stopped.")
            except subprocess.TimeoutExpired:
                print(f"Timeout expired while stopping '{self.capability_node.name}', killing it.")
                self.process.kill()
                self.process.wait()
            except Exception as e:
                print(f"Error stopping capability '{self.capability_node.name}': {e}")
            finally:
                self.process = None  # Clean up the process object
        else:
            print(f"Capability '{self.capability_node.name}' is not running.")

    def get_output(self):
        """Retrieve the most recent output line."""
        rlist, _, _ = select.select([self.master_fd], [], [], 0)  # 0-second timeout, return immediately
        if rlist:
            try:
                data = os.read(self.master_fd, self.max_output)
                return data.decode('utf-8', 'ignore')
            except OSError:
                return ''
        else:
            return ''  # No data available

    def is_running(self):
        """Check whether the process is still running."""
        return self.process and self.process.poll() is None


class CapabilityManager:
    """
    Manages the lifecycle and processes of a group of CapabilityNode objects.
    """
    def __init__(self, project_root_path: str):
        self.project_root_path = project_root_path
        # Store all discovered CapabilityNode objects
        self.available_capabilities: dict[str, CapabilityNode] = {
            node.name: node for node in get_capability_details(project_root_path)
        }
        # Store all currently running CapabilityProcess instances
        self.running_processes: dict[str, CapabilityProcess] = {}
        print(f"Initialized CapabilityManager. Found {len(self.available_capabilities)} capabilities.")

    def __del__(self):
        """When the manager object is destroyed, stop all active processes."""
        print("CapabilityManager is being destroyed. Stopping all running capabilities...")
        self.stop_all_capabilities()

    def get_capability_node(self, capability_name: str) -> CapabilityNode | None:
        """Retrieve a CapabilityNode object by ID."""
        return self.available_capabilities.get(capability_name)

    def print_available_capabilities(self):
        """Print all available capabilities and their info."""
        if not self.available_capabilities:
            print("No capabilities found.")
            return

        print("\n--- Available Capabilities ---")
        for cap_id, cap_node in self.available_capabilities.items():
            status = "Running" if cap_id in self.running_processes and self.running_processes[cap_id].is_running() else "Stopped"
            print(f"- [{status}] {cap_node.name} (dir: {cap_node.cwd}, version: {cap_node.version}, start_on_boot: {cap_node.start_on_boot})")
            if cap_node.startup_command:
                print(f"  Command: '{cap_node.startup_command}'")
            else:
                print(f"  No startup command defined.")

    def start_capability(self, capability_name: str):
        """Start a capability by ID."""
        node = self.available_capabilities.get(capability_name)
        if not node:
            print(f"Error: Capability '{capability_name}' not found.")
            return False

        if node.name in self.running_processes and self.running_processes[node.name].is_running():
            print(f"Capability '{node.name}' is already running.")
            return True

        print(f"Attempting to start capability: {node.name}")
        process_wrapper = CapabilityProcess(node, self.project_root_path)
        if process_wrapper.start():
            self.running_processes[node.name] = process_wrapper
            return True
        else:
            print(f"Failed to start capability: {node.name}")
            return False

    def boot(self):
        """Start all capabilities that should start on boot."""
        for cap_id, cap_node in self.available_capabilities.items():
            if cap_node.start_on_boot and not cap_id in self.running_processes:
                self.start_capability(cap_id)

    def stop_capability(self, capability_name: str):
        """Stop a capability by ID."""
        process_wrapper = self.running_processes.get(capability_name)
        if process_wrapper:
            process_wrapper.stop()
            del self.running_processes[capability_name]  # Remove from manager
            return True
        else:
            print(f"Capability '{capability_name}' is not running.")
            return False

    def stop_all_capabilities(self):
        """Stop all currently running capabilities."""
        to_stop = list(self.running_processes.keys())  # Create a copy since the dictionary will be modified
        for cap_id in to_stop:
            self.stop_capability(cap_id)
        print("All running capabilities stopped.")

    # TODO: implement specific output for different streams (stdout, stderr, etc.)
    def print_capability_output(self, capability_name: str, stream="stdout"):
        """Print the most recent output from the specified capability."""
        process_wrapper = self.running_processes.get(capability_name)
        if process_wrapper:
            output = process_wrapper.get_output()
            print(output)
        else:
            print(f"Capability '{capability_name}' is not running, so no output to display.")

    def get_all_running_PIDs(self):
        """Get the PIDs of all currently running capabilities."""
        pids = {}
        for cap_id, process_wrapper in self.running_processes.items():
            if process_wrapper.is_running():
                pids[cap_id] = process_wrapper.process.pid
        return pids

if __name__ == "__main__":
    import time
    # Assuming the project root directory is ./
    # Please adjust `project_root_path` to your actual project root.
    # For demonstration, use current working directory as project root.
    project_root_path = os.getcwd() 
    print(f"Assumed project root path: {project_root_path}")

    # For testing, ensure your project root has the following structure:
    # {project_root_path}/
    # └── capability/
    #     ├── example_cap/          <-- Capability directory (name will be read from description.yml)
    #     │   └── description.yml
    #     │   └── test_script.py    (a Python script that endlessly prints to stdout/stderr)
    #     └── ping/                 <-- Example capability (name from description.yml might be "ping")
    #         └── description.yml
    #         └── ping_script.sh    (a shell script that endlessly pings)

    manager = CapabilityManager(project_root_path)
    manager.print_available_capabilities()

    # == Demonstrating Start, Stop, and Output Retrieval ==

    # 1. Start a specific capability (assuming 'ping' is its recorded name from description.yml)
    print("\n--- Starting a 'ping' capability explicitly ---")
    manager.start_capability("ping") # Use the 'name' field from description.yml
    
    # 2. Boot up all capabilities marked `start_on_boot`
    manager.boot()

    print("\n--- Current running PIDs after initial start/boot ---")
    pids = manager.get_all_running_PIDs()
    print(f"Running PIDs: {pids}")

    manager.print_available_capabilities()

    time.sleep(5) # Wait for some output to accumulate

    # 3. Print output from specific capabilities
    print("\n--- Printing output for 'example' and 'ping' ---")
    manager.print_capability_output("example")
    manager.print_capability_output("ping")

    # 4. Stop a specific capability
    print("\n--- Stopping 'example' ---")
    manager.stop_capability("example")
    time.sleep(1) # Give it a moment to stop

    # 5. Stop all remaining running capabilities
    print("\n--- Stopping all remaining capabilities ---")
    manager.stop_all_capabilities()

    # 6. Verify no PIDs are running
    pids = manager.get_all_running_PIDs()
    print(f"Running PIDs after stopping all: {pids}")

    print("\nScript finished. The CapabilityManager's __del__ method will clean up any remaining processes.")
    # Explicitly deleting the manager (optional, __del__ is usually called on program exit)
    # del manager
