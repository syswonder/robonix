# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025 syswonder
from datetime import datetime
import readline
import sys
import os
import atexit
from typing import List, Dict, Any, Optional, Callable
import traceback
import aioconsole

class Colors:
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"
    GRAY = "\033[90m"
    RESET = "\033[0m"
    BOLD = "\033[1m"


def _color_print(text, color_code, bold=False, end="\n"):
    style = Colors.BOLD if bold else ""
    sys.stdout.write(f"{style}{color_code}{text}{Colors.RESET}{end}")
    sys.stdout.flush()


def print_red(text, bold=False, end="\n"):
    _color_print(text, Colors.RED, bold, end)


def print_green(text, bold=False, end="\n"):
    _color_print(text, Colors.GREEN, bold, end)


def print_yellow(text, bold=False, end="\n"):
    _color_print(text, Colors.YELLOW, bold, end)


def print_blue(text, bold=False, end="\n"):
    _color_print(text, Colors.BLUE, bold, end)


def print_magenta(text, bold=False, end="\n"):
    _color_print(text, Colors.MAGENTA, bold, end)


def print_cyan(text, bold=False, end="\n"):
    _color_print(text, Colors.CYAN, bold, end)


def print_gray(text, bold=False, end="\n"):
    _color_print(text, Colors.GRAY, bold, end)


class Command:
    """Simple command definition"""

    def __init__(self, name: str, description: str, handler: Callable, usage: str = ""):
        self.name = name
        self.description = description
        self.handler = handler
        self.usage = usage or name


class CommandRegistry:
    """Simple command registry"""

    def __init__(self):
        self.commands: Dict[str, Command] = {}

    def register_command(self, command: Command):
        """Register command"""
        self.commands[command.name] = command

    def get_command(self, name: str) -> Optional[Command]:
        """Get command"""
        return self.commands.get(name)

    def get_all_command_names(self) -> List[str]:
        """Get all command names"""
        return list(self.commands.keys())


class Completer:
    """Enhanced autocompleter for command names and subcommands"""

    def __init__(self, command_registry: CommandRegistry, manager):
        self.command_registry = command_registry
        self.manager = manager

    def complete(self, text: str, state: int) -> Optional[str]:
        """Autocompletion callback function for commands and subcommands"""
        if state == 0:
            # Get current input line
            line = readline.get_line_buffer()
            words = line.split()
            
            if len(words) <= 1:
                # First word or blank input: complete command names
                all_commands = self.command_registry.get_all_command_names()
                self.matches = [cmd for cmd in all_commands if cmd.startswith(text)]
            elif len(words) == 2:
                # Second word: complete subcommands based on command
                command_name = words[0]
                if command_name == "list":
                    # Subcommands for list command are node names
                    all_nodes = list(self.manager.available_nodes.keys())
                    self.matches = [node for node in all_nodes if node.startswith(text)]
                elif command_name in ["start", "stop", "output"]:
                    # Subcommands for start/stop/output commands are also node names
                    all_nodes = list(self.manager.available_nodes.keys())
                    self.matches = [node for node in all_nodes if node.startswith(text)]
                else:
                    self.matches = []
            else:
                self.matches = []

        if state < len(self.matches):
            return self.matches[state]
        else:
            return None


class CLI:
    """Command Line Interface for Robonix"""

    """
        How to add a new command:
        1. add a new method similar to _cmd_list, _cmd_start, .. in this class
        2. register it in _register_builtin_commands

        All commands will be automatically managed by CLI with auto-completion and command history feature.
    """

    def __init__(self, manager):
        self.manager = manager
        self.command_registry = CommandRegistry()
        self.completer = Completer(self.command_registry, manager)

        self.history_file = os.path.expanduser("~/.robonix_history")

        readline.set_completer(self.completer.complete)
        readline.parse_and_bind("tab: complete")

        try:
            readline.read_history_file(self.history_file)
        except FileNotFoundError:
            pass

        atexit.register(self._save_history)

        self._register_builtin_commands()

    def _save_history(self):
        """Save command history to file"""
        try:
            readline.write_history_file(self.history_file)
        except Exception:
            pass

    def _register_builtin_commands(self):
        """Register built-in commands"""
        commands = [
            Command(
                name="list",
                description="List all available nodes",
                handler=self._cmd_list,
                usage="list [node_name]",
            ),
            Command(
                name="start",
                description="Start a node",
                handler=self._cmd_start,
                usage="start <node_name>",
            ),
            Command(
                name="stop",
                description="Stop a node",
                handler=self._cmd_stop,
                usage="stop <node_name>",
            ),
            Command(
                name="output",
                description="Show output from a running node",
                handler=self._cmd_output,
                usage="output <node_name>",
            ),
            Command(
                name="pids",
                description="Show all running process PIDs",
                handler=self._cmd_pids,
            ),
            Command(
                name="help",
                description="Show help information",
                handler=self._cmd_help,
                usage="help [command]",
            ),
            Command(
                name="history",
                description="Show command history",
                handler=self._cmd_history,
                usage="history [number]",
            ),
            Command(
                name="mcp_log",
                description="Show MCP server logs",
                handler=self._cmd_mcp_log,
                usage="mcp_log [lines]",
            ),
            Command(
                name="exit", description="Exit the application", handler=self._cmd_exit
            ),
        ]

        for cmd in commands:
            self.command_registry.register_command(cmd)

    def _cmd_list(self, args: List[str]) -> bool:
        """List available nodes command"""
        if len(args) == 0:
            # 显示简洁的节点列表
            self._print_nodes_summary()
        else:
            # 显示特定节点的详细信息
            node_name = args[0]
            self._print_node_details(node_name)
        return True

    def _print_nodes_summary(self):
        """Print a concise summary of all nodes"""
        if not self.manager.available_nodes:
            print_yellow("No nodes found.")
            return

        print_cyan("Available nodes:", bold=True)
        print("-" * 60)
        
        # Group by status
        running_nodes = []
        stopped_nodes = []
        
        for node_id, node in self.manager.available_nodes.items():
            is_running = (node_id in self.manager.running_processes and 
                         self.manager.running_processes[node_id].is_running())
            if is_running:
                running_nodes.append(node)
            else:
                stopped_nodes.append(node)
        
        # Group nodes by type
        driver_nodes = []
        capability_nodes = []
        other_nodes = []
        
        for node_id, node in self.manager.available_nodes.items():
            if node.node_type == "driver":
                driver_nodes.append((node_id, node))
            elif node.node_type == "capability":
                capability_nodes.append((node_id, node))
            else:
                other_nodes.append((node_id, node))
        
        # Helper function to display node groups
        def print_node_group(nodes, group_name=None):
            if group_name:
                print_cyan(f"\n{group_name}:", bold=True)
                print("-" * 40)
            
            for node_id, node in nodes:
                is_running = (node_id in self.manager.running_processes and 
                             self.manager.running_processes[node_id].is_running())
                
                if is_running:
                    status = f"{Colors.GREEN}Running{Colors.RESET}"
                    status_icon = f"{Colors.GREEN}✓{Colors.RESET}"
                else:
                    status = f"{Colors.RED}Stopped{Colors.RESET}"
                    status_icon = f"{Colors.RED}✗{Colors.RESET}"
                
                # Extract directory name from cwd path
                import os
                cwd_parts = os.path.normpath(node.cwd).split(os.sep)
                folder_name = ""
                if len(cwd_parts) > 0:
                    folder_name = cwd_parts[-1]  # Get last directory name
                
                # Format display based on node type
                node_type_display = ""
                if node.node_type and folder_name:
                    if node.node_type == "driver":
                        node_type_display = f" {Colors.GRAY}(driver@{folder_name}){Colors.RESET}"
                    elif node.node_type == "capability":
                        node_type_display = f" {Colors.GRAY}(cap@{folder_name}){Colors.RESET}"
                    else:
                        node_type_display = f" {Colors.GRAY}({node.node_type}@{folder_name}){Colors.RESET}"
                
                # Format display: status icon + status + node name (white bold) + type info (gray)
                node_name_display = f"{Colors.BOLD}{Colors.WHITE}{node.name}{Colors.RESET}"
                print(f"{status_icon} [{status:<8}] {node_name_display}{node_type_display}")
        
        # Display node groups in order
        if driver_nodes:
            print_node_group(driver_nodes, "Driver Nodes")
        
        if capability_nodes:
            if driver_nodes:  # Add separator if there are driver nodes before
                print()
            print_node_group(capability_nodes, "Capability Nodes")
        
        if other_nodes:
            if driver_nodes or capability_nodes:  # Add separator if there are other nodes before
                print()
            print_node_group(other_nodes, "Other Nodes")
        
        print("-" * 60)
        print(f"Total: {len(running_nodes)} running, {len(stopped_nodes)} stopped")
        print_cyan("Tip: Use 'list <node_name>' to see detailed information for a specific node", bold=False)

    def _print_node_details(self, node_name: str):
        """Print detailed information for a specific node"""
        node = self.manager.available_nodes.get(node_name)
        if not node:
            print_red(f"Error: Node '{node_name}' not found.")
            print("Available nodes:", ", ".join(self.manager.available_nodes.keys()))
            return
        
        # Check running status
        is_running = (node_name in self.manager.running_processes and 
                     self.manager.running_processes[node_name].is_running())
        status = "Running" if is_running else "Stopped"
        status_color = Colors.GREEN if is_running else Colors.RED
        
        print_cyan(f"Node Details: {node.name}", bold=True)
        print("-" * 50)
        print(f"Status: {status_color}{status}{Colors.RESET}")
        print(f"Version: {node.version}")
        print(f"Directory: {node.cwd}")
        print(f"Start on boot: {node.start_on_boot}")
        
        if node.startup_command:
            print(f"Command: {node.startup_command}")
        else:
            print("Command: No startup command defined")
        
        if is_running:
            process = self.manager.running_processes[node_name]
            if process.process:
                print(f"PID: {process.process.pid}")

    def _cmd_start(self, args: List[str]) -> bool:
        """Start node command"""
        if len(args) < 1:
            print_red("Error: Missing node name.")
            print("Usage: start <node_name>")
            return False

        node_name = args[0]
        return self.manager.start_node(node_name)

    def _cmd_stop(self, args: List[str]) -> bool:
        """Stop node command"""
        if len(args) < 1:
            print_red("Error: Missing node name.")
            print("Usage: stop <node_name>")
            return False

        node_name = args[0]
        return self.manager.stop_node(node_name)

    def _cmd_output(self, args: List[str]) -> bool:
        """Show node output command"""
        if len(args) < 1:
            print_red("Error: Missing node name.")
            print("Usage: output <node_name>")
            return False

        node_name = args[0]
        self.manager.print_node_output(node_name)
        return True

    def _cmd_pids(self, args: List[str]) -> bool:
        """Show running PIDs command"""
        pids = self.manager.get_all_running_PIDs()
        print_magenta(f"Running PIDs: {pids}")
        return True

    def _cmd_help(self, args: List[str]) -> bool:
        """Help command"""
        if len(args) == 0:
            self._show_general_help()
        else:
            command_name = args[0]
            self._show_command_help(command_name)
        return True

    def _show_general_help(self):
        """Show general help information"""
        print_cyan("Available commands:", bold=True)
        print()

        for cmd in self.command_registry.commands.values():
            print(f"  {cmd.name:<15} - {cmd.description}")

        print()
        print(
            "Type 'help <command>' for detailed information about a specific command."
        )

    def _show_command_help(self, command_name: str):
        """Show help for specific command"""
        cmd = self.command_registry.get_command(command_name)
        if not cmd:
            print_red(f"Unknown command: {command_name}")
            return

        print_cyan(f"Command: {cmd.name}", bold=True)
        print(f"Description: {cmd.description}")
        print(f"Usage: {cmd.usage}")

    def _cmd_history(self, args: List[str]) -> bool:
        """Show command history"""
        try:
            if len(args) > 0:
                num_lines = int(args[0])
            else:
                num_lines = 20

            history_length = readline.get_current_history_length()
            start_idx = max(1, history_length - num_lines + 1)

            for i in range(start_idx, history_length + 1):
                cmd = readline.get_history_item(i)
                if cmd:
                    print(f"{i:4d}  {cmd}")
        except ValueError:
            print_red("Error: Invalid number for history command")
        except Exception as e:
            print_red(f"Error showing history: {e}")

        return True

    def _cmd_mcp_log(self, args: List[str]) -> bool:
        """Show MCP server logs"""
        try:
            if len(args) > 0:
                num_lines = int(args[0])
            else:
                num_lines = 20

            log_file = os.path.expanduser("~/.robonix_mcp.log")
            
            if not os.path.exists(log_file):
                print_yellow("MCP log file not found. MCP server may not be running.")
                return True
            
            with open(log_file, 'r') as f:
                lines = f.readlines()
                total_lines = len(lines)
                start_idx = max(0, total_lines - num_lines)
                
                print_cyan(f"MCP Server Log (last {num_lines} lines):", bold=True)
                print("-" * 50)
                
                for i in range(start_idx, total_lines):
                    print(lines[i].rstrip())
                    
                if total_lines > num_lines:
                    print(f"\n... ({total_lines - num_lines} more lines, use 'mcp_log {total_lines}' to see all)")
                    
        except ValueError:
            print_red("Error: Invalid number for mcp_log command")
        except Exception as e:
            print_red(f"Error reading MCP log: {e}")

        return True

    def _cmd_exit(self, args: List[str]) -> bool:
        """Exit command"""
        self.manager.stop_all_nodes()
        return False

    def register_custom_command(self, command: Command):
        """Register custom command"""
        self.command_registry.register_command(command)

    def get_ros_info(self):
        """Get ROS info"""
        return (
            "ROS_DISTRO: "
            + (os.getenv("ROS_DISTRO") or "N/A")
            + ", ROS_VERSION: "
            + (os.getenv("ROS_VERSION") or "N/A")
        )

    def get_user_info(self):
        """Get user info"""
        try:
            return "user: " + os.getlogin()
        except Exception:
            return "user: N/A"


    def run(self):
        """Run command line interface"""
        print_cyan(
            "Welcome to Robonix Shell (system time: "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            + ", "
            + self.get_user_info()
            + ", "
            + self.get_ros_info()
            + ")",
            bold=True,
        )
        print(
            "Type 'help' for available commands, you can use TAB for command auto-completion"
        )

        readline.set_history_length(1000)
        readline.set_auto_history(True)

        try:
            readline.read_init_file()
        except FileNotFoundError:
            readline.parse_and_bind("set editing-mode emacs")
            readline.parse_and_bind("set completion-ignore-case on")
            readline.parse_and_bind("set show-all-if-ambiguous on")
            readline.parse_and_bind("set completion-display-width 0")
            readline.parse_and_bind("set horizontal-scroll-mode on")
            readline.parse_and_bind("set bell-style none")
            readline.parse_and_bind("set colored-completion-prefix on")
            readline.parse_and_bind("set colored-stats on")
            readline.parse_and_bind("set skip-completed-text on")
            readline.parse_and_bind("set menu-complete-display-prefix on")
            readline.parse_and_bind("set expand-tilde on")
            readline.parse_and_bind("set mark-directories on")
            readline.parse_and_bind("set mark-symlinked-directories on")
            readline.parse_and_bind("set match-hidden-files on")
            readline.parse_and_bind("set visible-stats on")

        def get_prompt():
            return f"{Colors.GREEN}{Colors.BOLD}> {Colors.RESET}"

        while True:
            try:
                prompt = get_prompt()
                try:
                    # Use input() instead of aioconsole.ainput() to preserve readline completion
                    line = input(prompt)
                except (EOFError, KeyboardInterrupt):
                    print_red("\nExiting...")
                    break
                line = line.strip()
                if not line:
                    continue
                if line and line != readline.get_history_item(
                    readline.get_current_history_length()
                ):
                    readline.add_history(line)
                parts = line.split()
                command_name = parts[0]
                args = parts[1:] if len(parts) > 1 else []
                command = self.command_registry.get_command(command_name)
                if not command:
                    print_red(f"Error: Unknown command '{command_name}'")
                    print(
                        "available commands: "
                        + ", ".join(self.command_registry.get_all_command_names())
                    )
                    print("Type 'help' for more information")
                    continue
                try:
                    result = command.handler(args)
                    if result is False:
                        break
                except Exception as e:
                    print_red(f"Error executing command '{command_name}': {e}")
                    print_red(traceback.format_exc())

            except KeyboardInterrupt:
                print_red("\nUse 'exit' to quit the application.")
                continue

        print("Goodbye!")
