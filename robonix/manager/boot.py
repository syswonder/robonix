import asyncio
import signal
import argparse
from robonix.manager import process_manage
from robonix.manager import node
from robonix.manager.log import logger
from robonix.manager.cmdline import CLI
from robonix.manager import depend
import os
from robonix.manager.constant import BASE_SKILL_PATH, INIT_FILE, BASE_PATH
import sys

from robonix.manager.eaios_decorators import package_init

# Ensure the root directory is in the Python path for skill import
# import skill


async def shutdown(signame):
    """Handle shutdown signals"""
    logger.info(f"Received {signame}, shutting down...")
    if "manager" in globals():
        manager.stop_all_nodes()
    loop = asyncio.get_event_loop()
    loop.stop()


async def main():

    manager.boot()
    try:
        # Redirect MCP server output to log file to avoid conflicts with CLI output
        import subprocess
        import sys

        # Create log file
        log_file = os.path.expanduser("~/.robonix_mcp.log")

        # Start MCP server process, redirect output to log file
        mcp_process = subprocess.Popen(
            [
                sys.executable,
                "-c",
                "import asyncio; import sys; import os; sys.path.append(os.path.dirname(os.path.abspath('.'))); from manager.eaios_decorators import mcp_start; asyncio.run(mcp_start())",
            ],
            stdout=open(log_file, "w"),
            stderr=subprocess.STDOUT,
            text=True,
        )

        # Wait a moment for MCP server to start
        await asyncio.sleep(1)

        cli = CLI(manager)

        # Run CLI in separate thread to avoid blocking async tasks
        import threading

        cli_thread = threading.Thread(target=cli.run, daemon=True)
        cli_thread.start()

        # Wait for CLI thread to finish
        cli_thread.join()

        # Clean up MCP process
        mcp_process.terminate()
        mcp_process.wait()

    except KeyboardInterrupt:
        logger.info("Exiting...")
    finally:
        manager.stop_all_nodes()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="eaios boot and args")
    parser.add_argument(
        "--config", type=str, required=True, help="Path to the configuration file"
    )
    args = parser.parse_args()
    node_list = node.get_node_details(args.config)
    depend.check_depend(args.config)
    manager = process_manage.RuntimeManager(node_list)
    package_init(args.config)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    # Signal handling for Unix-like systems
    for signame in ("SIGINT", "SIGTERM"):
        try:
            loop.add_signal_handler(
                getattr(signal, signame), lambda: asyncio.create_task(shutdown(signame))
            )
        except NotImplementedError:
            # If platform doesn't support signal handling, use fallback
            pass

    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        logger.info("Received KeyboardInterrupt, shutting down...")
    finally:
        # Clean up resources
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()
