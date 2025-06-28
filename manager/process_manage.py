from node import *
import os
import subprocess
import threading
import pty
import select
import signal

# NodeProcess class
class ProcessNode:
    """
    Ennodesulates a running subprocess and manages its input/output.
    """
    def __init__(self, node_node: BaseNode, max_output: int = 1_000_000):
        self.node_node = node_node
        self.process = None  # Subprocess Popen object
        self.max_output = max_output
        self.master_fd, self.slave_fd = pty.openpty()
        self._running = False

    def start(self):
        """Start the process associated with the node."""
        if self.process and self.process.poll() is None:
            print(f"Node '{self.node_node.name}' is already running.")
            return True

        command = self.node_node.startup_command
        if not command:
            print(f"Error: No startup command defined for '{self.node_node.name}'.")
            return False

        # Construct the process working directory
        process_cwd = self.node_node.cwd
        if not os.path.exists(process_cwd):
            print(f"Error: Process working directory '{process_cwd}' does not exist for '{self.node_node.name}'.")
            return False

        try:
            # shell=True allows running shell commands with pipes or compound commands, but has security risks
            # Prefer shell=False and split the command into a list if possible
            # Here we assume startup_command is safe and needs to be run in a shell
            self.process = subprocess.Popen(
                "stdbuf -oL " + command,
                shell=True,
                cwd=process_cwd,  # Run in the directory specific to this node
                stdin=self.slave_fd,
                stdout=self.slave_fd,
                stderr=subprocess.STDOUT,
                text=True,  # Ensure text mode input/output (Python 3.6+)
                bufsize=1,  # Line buffering
                universal_newlines=True  # Legacy compatibility, same as text=True
            )
            
            self._running = True
            print(f"Started node '{self.node_node.name}' with PID: {self.process.pid} from '{process_cwd}'.")
            return True
        except FileNotFoundError:
            print(f"Error: Command '{command.split()[0]}' not found for '{self.node_node.name}'.")
            return False
        except Exception as e:
            print(f"Error starting node '{self.node_node.name}': {e}")
            return False

    def stop(self, timeout=15):
        """Stop the process associated with the node."""
        if self.process and self.process.poll() is None:  # poll() is None means the process is still running
            print(f"Stopping node '{self.node_node.name}' (PID: {self.process.pid})...")
            try:
                self._running = False  # Notify the reading thread to exit
                self.process.send_signal(sig=signal.SIGINT)  # Send SIGTERM
                self.process.wait(timeout=timeout)  # Wait for graceful shutdown

                if self.process.poll() is None:  # If the process hasn't terminated
                    print(f"Node '{self.node_node.name}' did not terminate gracefully, killing it.")
                    self.process.kill()  # Send SIGKILL to forcefully terminate
                    self.process.wait()  # Ensure the process is completely terminated
                print(f"Node '{self.node_node.name}' stopped.")
            except subprocess.TimeoutExpired:
                print(f"Timeout expired while stopping '{self.node_node.name}', killing it.")
                self.process.kill()
                self.process.wait()
            except Exception as e:
                print(f"Error stopping node '{self.node_node.name}': {e}")
            finally:
                self.process = None  # Clean up the process object
        else:
            print(f"Node '{self.node_node.name}' is not running.")

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


class RuntimeManager:
    """
    Manages the lifecycle and processes of a group of BaseNode objects.
    """
    def __init__(self, nodes:list[BaseNode]):
        # Store all discovered BaseNode objects
        self.available_nodes: dict[str, BaseNode] = {
            node.name: node for node in nodes
        }
        # Store all currently running NodeProcess instances
        self.running_processes: dict[str, ProcessNode] = {}
        print(f"Initialized NodeManager. Found {len(self.available_nodes)} nodes.")

    def __del__(self):
        """When the manager object is destroyed, stop all active processes."""
        print("NodeManager is being destroyed. Stopping all running nodes...")
        self.stop_all_nodes()

    def get_node_node(self, node_name: str) -> BaseNode | None:
        """Retrieve a BaseNode object by ID."""
        return self.available_nodes.get(node_name)

    def print_available_nodes(self):
        """Print all available nodes and their info."""
        if not self.available_nodes:
            print("No nodes found.")
            return

        print("\n--- Available nodes ---")
        for node_id, node_node in self.available_nodes.items():
            status = "Running" if node_id in self.running_processes and self.running_processes[node_id].is_running() else "Stopped"
            print(f"- [{status}] {node_node.name} (dir: {node_node.cwd}, version: {node_node.version}, start_on_boot: {node_node.start_on_boot})")
            if node_node.startup_command:
                print(f"  Command: '{node_node.startup_command}'")
            else:
                print(f"  No startup command defined.")

    def start_node(self, node_name: str):
        """Start a node by ID."""
        node = self.available_nodes.get(node_name)
        if not node:
            print(f"Error: Node '{node_name}' not found.")
            return False

        if node.name in self.running_processes and self.running_processes[node.name].is_running():
            print(f"Node '{node.name}' is already running.")
            return True

        print(f"Attempting to start node: {node.name}")
        process_wrapper = ProcessNode(node)
        if process_wrapper.start():
            self.running_processes[node.name] = process_wrapper
            return True
        else:
            print(f"Failed to start node: {node.name}")
            return False

    def boot(self):
        """Start all nodes that should start on boot."""
        for node_id, node_node in self.available_nodes.items():
            if node_node.start_on_boot and not node_id in self.running_processes:
                self.start_node(node_id)

    def stop_node(self, node_name: str):
        """Stop a node by ID."""
        process_wrapper = self.running_processes.get(node_name)
        if process_wrapper:
            process_wrapper.stop()
            del self.running_processes[node_name]  # Remove from manager
            return True
        else:
            print(f"Node '{node_name}' is not running.")
            return False

    def stop_all_nodes(self):
        """Stop all currently running nodes."""
        to_stop = list(self.running_processes.keys())  # Create a copy since the dictionary will be modified
        for node_id in to_stop:
            self.stop_node(node_id)
        print("All running nodes stopped.")

    # TODO: implement specific output for different streams (stdout, stderr, etc.)
    def print_node_output(self, node_name: str, stream="stdout"):
        """Print the most recent output from the specified node."""
        process_wrapper = self.running_processes.get(node_name)
        if process_wrapper:
            while True:
                output = process_wrapper.get_output()
                if len(output)==0:
                    break
                print(output)
        else:
            print(f"Node '{node_name}' is not running, so no output to display.")

    def get_all_running_PIDs(self):
        """Get the PIDs of all currently running nodes."""
        pids = {}
        for node_id, process_wrapper in self.running_processes.items():
            if process_wrapper.is_running():
                pids[node_id] = process_wrapper.process.pid
        return pids

if __name__ == "__main__":
    import time
    # Assuming the project root directory is ./
    # Please adjust `target_path` to your actual project root.
    # For demonstration, use current working directory as project root.
    target_path = os.getcwd() 
    print(f"Assumed project root path: {target_path}")

    # For testing, ensure your project root has the following structure:
    # {target_path}/
    # └── node/
    #     ├── example_node/          <-- Node directory (name will be read from description.yml)
    #     │   └── description.yml
    #     │   └── test_script.py    (a Python script that endlessly prints to stdout/stderr)
    #     └── ping/                 <-- Example node (name from description.yml might be "ping")
    #         └── description.yml
    #         └── ping_script.sh    (a shell script that endlessly pings)

    manager = RuntimeManager(target_path)
    manager.print_available_nodes()

    # == Demonstrating Start, Stop, and Output Retrieval ==

    # 1. Start a specific node (assuming 'ping' is its recorded name from description.yml)
    print("\n--- Starting a 'ping' node explicitly ---")
    manager.start_node("ping") # Use the 'name' field from description.yml
    
    # 2. Boot up all nodes marked `start_on_boot`
    manager.boot()

    print("\n--- Current running PIDs after initial start/boot ---")
    pids = manager.get_all_running_PIDs()
    print(f"Running PIDs: {pids}")

    manager.print_available_nodes()

    time.sleep(5) # Wait for some output to accumulate

    # 3. Print output from specific nodes
    print("\n--- Printing output for 'example' and 'ping' ---")
    manager.print_node_output("example")
    manager.print_node_output("ping")

    # 4. Stop a specific node
    print("\n--- Stopping 'example' ---")
    manager.stop_node("example")
    time.sleep(1) # Give it a moment to stop

    # 5. Stop all remaining running nodes
    print("\n--- Stopping all remaining nodes ---")
    manager.stop_all_nodes()

    # 6. Verify no PIDs are running
    pids = manager.get_all_running_PIDs()
    print(f"Running PIDs after stopping all: {pids}")

    print("\nScript finished. The NodeManager's __del__ method will clean up any remaining processes.")
    # Explicitly deleting the manager (optional, __del__ is usually called on program exit)
    # del manager
