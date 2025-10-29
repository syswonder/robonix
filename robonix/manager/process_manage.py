from node import *
import os
import subprocess
import threading
import pty
import select
import signal
from log import logger
import shlex

def build_cli_args(params: dict, *, normalize_dash: bool = True) -> str:
    """
    combine params into a CLI argument string:
    - key -> '--key value'
    - True -> '--key'
    - False/None -> skip
    - list/tuple/set -> '--key v1 --key v2 ...'
    - dict -> '--key.sub value'
    - key underscore will be replaced with dash
    """
    def _k(k: str) -> str:
        k = k.replace('_', '-') if normalize_dash else k
        return k if k.startswith('-') else f'--{k}'

    def _flatten(flag: str, v):
        if v is True:
            yield (flag, None)
        elif v in (False, None):
            return
        elif isinstance(v, (list, tuple, set)):
            for it in v:
                # each element in the list/tuple/set will bind the same flag
                yield from _flatten(flag, it)
        elif isinstance(v, dict):
            for k2, v2 in v.items():
                yield from _flatten(f'{flag}.{k2.replace("_","-")}', v2)
        else:
            yield (flag, str(v))

    parts = []
    for k, v in (params or {}).items():
        flag = _k(k)
        for f, val in _flatten(flag, v):
            if val is None:
                parts.append(f)
            else:
                parts.append(f + ' ' + shlex.quote(val))
    return ' '.join(parts)

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
            logger.info(f"Node '{self.node_node.name}' is already running.")
            return True

        command = self.node_node.startup_command
        if not command:
            logger.error(
                f"Error: No startup command defined for '{self.node_node.name}'."
            )
            return False

        # Construct the process working directory
        process_cwd = self.node_node.cwd
        if not os.path.exists(process_cwd):
            logger.error(
                f"Error: Process working directory '{process_cwd}' does not exist for '{self.node_node.name}'."
            )
            return False

        try:
            # shell=True allows running shell commands with pipes or compound commands, but has security risks
            # Prefer shell=False and split the command into a list if possible
            # Here we assume startup_command is safe and needs to be run in a shell
            base = shlex.split(command)  # split command into a list
            param_list = shlex.split(build_cli_args(self.node_node.params)) if self.node_node.params else []
            argv = ["stdbuf", "-oL", *base, *param_list]
            self.process = subprocess.Popen(
                argv,
                shell=False,
                cwd=process_cwd,  # Run in the directory specific to this node
                stdin=self.slave_fd,
                stdout=self.slave_fd,
                stderr=subprocess.STDOUT,
                text=True,  # Ensure text mode input/output (Python 3.6+)
                bufsize=1,  # Line buffering
                universal_newlines=True,  # Legacy compatibility, same as text=True
            )

            self._running = True
            logger.info(
                f"Started node '{self.node_node.name}' with PID: {self.process.pid} from '{process_cwd}'."
            )
            return True
        except FileNotFoundError:
            logger.error(
                f"Error: Command '{command.split()[0]}' not found for '{self.node_node.name}'."
            )
            return False
        except Exception as e:
            logger.error(f"Error starting node '{self.node_node.name}': {e}")
            return False

    def stop(self, timeout=3):
        """Stop the process associated with the node."""
        if (
            self.process and self.process.poll() is None
        ):  # poll() is None means the process is still running
            logger.info(
                f"Stopping node '{self.node_node.name}' (PID: {self.process.pid})..."
            )
            try:
                self._running = False  # Notify the reading thread to exit
                self.process.send_signal(sig=signal.SIGINT)  # Send SIGTERM
                self.process.wait(timeout=timeout)  # Wait for graceful shutdown

                if self.process.poll() is None:  # If the process hasn't terminated
                    logger.warning(
                        f"Node '{self.node_node.name}' did not terminate gracefully, killing it."
                    )
                    self.process.kill()  # Send SIGKILL to forcefully terminate
                    self.process.wait()  # Ensure the process is completely terminated
                logger.info(f"Node '{self.node_node.name}' stopped.")
            except subprocess.TimeoutExpired:
                logger.warning(
                    f"Timeout expired while stopping '{self.node_node.name}', killing it."
                )
                self.process.kill()
                self.process.wait()
            except Exception as e:
                logger.error(f"Error stopping node '{self.node_node.name}': {e}")
            finally:
                self.process = None  # Clean up the process object
        else:
            logger.warning(f"Node '{self.node_node.name}' is not running.")

    def get_output(self):
        """Retrieve the most recent output line."""
        rlist, _, _ = select.select(
            [self.master_fd], [], [], 1
        )  # 0-second timeout, return immediately
        if rlist:
            try:
                data = os.read(self.master_fd, self.max_output)
                return data.decode("utf-8", "ignore")
            except OSError:
                return ""
        else:
            return ""  # No data available

    def is_running(self):
        """Check whether the process is still running."""
        return self.process and self.process.poll() is None


class RuntimeManager:
    """
    Manages the lifecycle and processes of a group of BaseNode objects.
    """

    def __init__(self, nodes: list[BaseNode]):
        # Store all discovered BaseNode objects
        self.available_nodes: dict[str, BaseNode] = {node.name: node for node in nodes}
        # Store all currently running NodeProcess instances
        self.running_processes: dict[str, ProcessNode] = {}
        logger.info(
            f"Initialized NodeManager. Found {len(self.available_nodes)} nodes."
        )

    def __del__(self):
        """When the manager object is destroyed, stop all active processes."""
        logger.info("NodeManager is being destroyed. Stopping all running nodes...")
        self.stop_all_nodes()

    def get_node_node(self, node_name: str) -> BaseNode | None:
        """Retrieve a BaseNode object by ID."""
        return self.available_nodes.get(node_name)

    def print_available_nodes(self):
        """Print all available nodes and their info."""
        if not self.available_nodes:
            logger.warning("No nodes found.")
            return

        logger.info("Available nodes:")

        def green_color(text):
            return "\033[92m" + text + "\033[0m"

        def red_color(text):
            return "\033[91m" + text + "\033[0m"

        def blue_bold_color(text):
            return "\033[94m\033[1m" + text + "\033[0m"

        for node_id, node_node in self.available_nodes.items():
            status = (
                green_color("Running")
                if node_id in self.running_processes
                and self.running_processes[node_id].is_running()
                else red_color("Stopped")
            )
            logger.info(
                f"- [{status}] {blue_bold_color(node_node.name)} (dir: {node_node.cwd}, version: {node_node.version}, start_on_boot: {node_node.start_on_boot})"
            )
            if node_node.startup_command:
                logger.info(f"  Command: '{node_node.startup_command}'")
            else:
                logger.info(f"  No startup command defined.")

    def start_node(self, node_name: str):
        """Start a node by ID."""
        node = self.available_nodes.get(node_name)
        if not node:
            logger.error(f"Error: Node '{node_name}' not found.")
            return False

        if (
            node.name in self.running_processes
            and self.running_processes[node.name].is_running()
        ):
            logger.info(f"Node '{node.name}' is already running.")
            return True

        logger.info(f"Attempting to start node: {node.name}")
        process_wrapper = ProcessNode(node)
        if process_wrapper.start():
            self.running_processes[node.name] = process_wrapper
            return True
        else:
            logger.error(f"Failed to start node: {node.name}")
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
            logger.warning(f"Node '{node_name}' is not running.")
            return False

    def stop_all_nodes(self):
        """Stop all currently running nodes."""
        to_stop = list(
            self.running_processes.keys()
        )  # Create a copy since the dictionary will be modified
        for node_id in to_stop:
            self.stop_node(node_id)
        logger.info("All running nodes stopped.")

    # TODO: implement specific output for different streams (stdout, stderr, etc.)
    def print_node_output(self, node_name: str, stream="stdout"):
        """Print the most recent output from the specified node."""
        process_wrapper = self.running_processes.get(node_name)
        if process_wrapper:
            while True:
                output = process_wrapper.get_output()
                if len(output) == 0:
                    break
                # Escape angle brackets to prevent Loguru colorization errors
                escaped_output = output.replace('<', '\\<').replace('>', '\\>')
                logger.info(escaped_output)
        else:
            logger.warning(
                f"Node '{node_name}' is not running, so no output to display."
            )

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
    logger.info(f"Assumed project root path: {target_path}")

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
    logger.info("\n--- Starting a 'ping' node explicitly ---")
    manager.start_node("ping")  # Use the 'name' field from description.yml

    # 2. Boot up all nodes marked `start_on_boot`
    manager.boot()

    logger.info("\n--- Current running PIDs after initial start/boot ---")
    pids = manager.get_all_running_PIDs()
    logger.info(f"Running PIDs: {pids}")

    manager.print_available_nodes()

    time.sleep(5)  # Wait for some output to accumulate

    # 3. Print output from specific nodes
    logger.info("\n--- Printing output for 'example' and 'ping' ---")
    manager.print_node_output("example")
    manager.print_node_output("ping")

    # 4. Stop a specific node
    logger.info("\n--- Stopping 'example' ---")
    manager.stop_node("example")
    time.sleep(1)  # Give it a moment to stop

    # 5. Stop all remaining running nodes
    logger.info("\n--- Stopping all remaining nodes ---")
    manager.stop_all_nodes()

    # 6. Verify no PIDs are running
    pids = manager.get_all_running_PIDs()
    logger.info(f"Running PIDs after stopping all: {pids}")

    logger.info(
        "\nScript finished. The NodeManager's __del__ method will clean up any remaining processes."
    )
    # Explicitly deleting the manager (optional, __del__ is usually called on program exit)
    # del manager
