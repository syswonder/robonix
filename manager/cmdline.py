# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025 syswonder
import readline
import sys
import os
import atexit
from typing import List, Dict, Any, Optional, Callable


class Colors:
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"
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


def print(text, bold=False, end="\n"):
    _color_print(text, Colors.YELLOW, bold, end)


def print_blue(text, bold=False, end="\n"):
    _color_print(text, Colors.BLUE, bold, end)


def print_magenta(text, bold=False, end="\n"):
    _color_print(text, Colors.MAGENTA, bold, end)


def print_cyan(text, bold=False, end="\n"):
    _color_print(text, Colors.CYAN, bold, end)


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
    """Simple autocompleter for command names only"""

    def __init__(self, command_registry: CommandRegistry):
        self.command_registry = command_registry

    def complete(self, text: str, state: int) -> Optional[str]:
        """Autocompletion callback function - only for command names"""
        if state == 0:

            all_commands = self.command_registry.get_all_command_names()
            self.matches = [cmd for cmd in all_commands if cmd.startswith(text)]

        if state < len(self.matches):
            return self.matches[state]
        else:
            return None


class CLI:
    """Command Line Interface for DeepEmbody"""

    """
        How to add a new command:
        1. add a new method similar to _cmd_list, _cmd_start, .. in this class
        2. register it in _register_builtin_commands

        All commands will be automatically managed by CLI with auto-completion and command history feature.
    """

    def __init__(self, manager):
        self.manager = manager
        self.command_registry = CommandRegistry()
        self.completer = Completer(self.command_registry)

        self.history_file = os.path.expanduser("~/.deepembody_history")

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
                name="exit", description="Exit the application", handler=self._cmd_exit
            ),
        ]

        for cmd in commands:
            self.command_registry.register_command(cmd)

    def _cmd_list(self, args: List[str]) -> bool:
        """List available nodes command"""
        self.manager.print_available_nodes()
        return True

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

    def _cmd_exit(self, args: List[str]) -> bool:
        """Exit command"""
        self.manager.stop_all_nodes()
        return False

    def register_custom_command(self, command: Command):
        """Register custom command"""
        self.command_registry.register_command(command)

    def run(self):
        """Run command line interface"""
        print_cyan("DeepEmbody Command Line Interface", bold=True)
        print(
            "Type 'help' for available commands, you can use TAB for command auto-completion"
        )

        readline.set_history_length(1000)
        readline.set_auto_history(True)

        try:
            readline.read_init_file()
        except FileNotFoundError:

            readline.parse_and_bind("tab: complete")
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

        readline.parse_and_bind("set colored-completion-prefix on")
        readline.parse_and_bind("set colored-stats on")

        while True:
            try:
                prompt = get_prompt()
                try:
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

            except KeyboardInterrupt:
                print_red("\nUse 'exit' to quit the application.")
                continue

        print("Goodbye!")
