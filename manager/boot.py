import asyncio
import signal
import process_manage
import node
from log import logger
from cmdline import CLI
import depend
import os
from constant import BASE_SKILL_PATH, INIT_FILE, BASE_PATH

if os.path.dirname(BASE_PATH) not in sys.path:
    sys.path.append(os.path.dirname(BASE_PATH))
from DeepEmbody.manager.eaios_decorators import package_init, mcp_start
import DeepEmbody.skill
logger.remove()
logger.add(
    sys.stderr,
    format="[{elapsed} <green>{name}</green>] {message}",
    level="INFO",
    colorize=True,
    backtrace=True,
    diagnose=True,
)
class Colors:
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

def _color_print(text, color_code, bold=False, end='\n'):
    style = Colors.BOLD if bold else ''
    sys.stdout.write(f"{style}{color_code}{text}{Colors.RESET}{end}")
    sys.stdout.flush()

def print_red(text, bold=False, end='\n'):
    _color_print(text, Colors.RED, bold, end)

def print_green(text, bold=False, end='\n'):
    _color_print(text, Colors.GREEN, bold, end)

def print_yellow(text, bold=False, end='\n'):
    _color_print(text, Colors.YELLOW, bold, end)

def print_blue(text, bold=False, end='\n'):
    _color_print(text, Colors.BLUE, bold, end)

def print_magenta(text, bold=False, end='\n'):
    _color_print(text, Colors.MAGENTA, bold, end)

def print_cyan(text, bold=False, end='\n'):
    _color_print(text, Colors.CYAN, bold, end)

def cmdline(manager: process_manage.RuntimeManager):
    """Entry point for command line interface."""
    while True:
        print_green(">", bold=True, end="")
        cmd = input().strip().split()
        if len(cmd) == 0:
            continue

        if cmd[0] == "list":
            manager.print_available_nodes()
        elif cmd[0] == "start":
            if len(cmd) < 2:
                print_red("Error: Missing node name.")
            node_name = cmd[1]
            manager.start_node(node_name)
        elif cmd[0] == "stop":
            if len(cmd) < 2:
                print_red("Error: Missing node name.")
            node_name = cmd[1]
            manager.stop_node(node_name)
        elif cmd[0] == "output":
            if len(cmd) < 2:
                print_red("Error: Missing node name.")
            node_name = cmd[1]
            manager.print_node_output(node_name)
        elif cmd[0] == "pids":
            pids = manager.get_all_running_PIDs()
            print_magenta(f"Running PIDs: {pids}")
        elif cmd[0] == "exit":
            manager.stop_all_nodes()
            break
        else:
            print_red(f"Error: Unknown command: {cmd}")
        
async def main():
    
    manager.boot()
    try:
        mcp_task = asyncio.create_task(mcp_start())
        # cmdline(manager)

        cli = CLI(manager)
        input_task = asyncio.create_task(cli.run())
        
        # 等待任意任务完成
        done, pending = await asyncio.wait(
            [mcp_task, input_task],
            return_when=asyncio.FIRST_COMPLETED
        )

        # 取消所有仍在运行的任务
        for task in pending:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass


    except KeyboardInterrupt:
        logger.info("Exiting...")
    finally:
        manager.stop_all_nodes()


if __name__ == "__main__":
    node_list = node.get_node_details("config/include.yml")
    depend.check_depend("config/include.yml")
    manager = process_manage.RuntimeManager(
        node_list
    )
    package_init("config/include.yml")
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    # Unix-like系统的信号处理
    for signame in ('SIGINT', 'SIGTERM'):
        try:
            loop.add_signal_handler(
                getattr(signal, signame),
                lambda: asyncio.create_task(shutdown(signame)))
        except NotImplementedError:
            # 如果平台不支持信号处理，使用备选方案
            pass

    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        print("\nReceived KeyboardInterrupt, shutting down...")
    finally:
        # 清理资源
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()
