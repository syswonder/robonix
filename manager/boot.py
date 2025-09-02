import asyncio
import signal
import argparse
import process_manage
import node
from log import logger
from cmdline import CLI
import depend
import os
from constant import BASE_SKILL_PATH, INIT_FILE, BASE_PATH
import sys

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
        # 重定向MCP服务器输出到日志文件，避免与CLI输出冲突
        import subprocess
        import sys
        
        # 创建日志文件
        log_file = os.path.expanduser("~/.deepembody_mcp.log")
        
        # 启动MCP服务器进程，重定向输出到日志文件
        mcp_process = subprocess.Popen(
            [sys.executable, "-c", 
             "import asyncio; from manager.eaios_decorators import mcp_start; asyncio.run(mcp_start())"],
            stdout=open(log_file, 'w'),
            stderr=subprocess.STDOUT,
            text=True
        )
        
        # 等待一下让MCP服务器启动
        await asyncio.sleep(1)
        
        cli = CLI(manager)
        
        # 在单独的线程中运行CLI以避免阻塞异步任务
        import threading
        cli_thread = threading.Thread(target=cli.run, daemon=True)
        cli_thread.start()
        
        # 等待CLI线程结束
        cli_thread.join()
        
        # 清理MCP进程
        mcp_process.terminate()
        mcp_process.wait()


    except KeyboardInterrupt:
        logger.info("Exiting...")
    finally:
        manager.stop_all_nodes()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='eaios boot and args')
    parser.add_argument("--config", type=str, required=True, help="Path to the configuration file")
    args = parser.parse_args()
    node_list = node.get_node_details(args.config)
    depend.check_depend(args.config)
    manager = process_manage.RuntimeManager(
        node_list
    )
    package_init(args.config)
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
