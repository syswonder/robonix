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

from eaios_decorators import package_init, mcp_start

# Ensure the root directory is in the Python path for skill import
sys.path.insert(0, BASE_PATH)
import skill

logger.remove()
logger.add(
    sys.stderr,
    format="[{elapsed} <green>{name}</green>] {message}",
    level="INFO",
    colorize=True,
    backtrace=True,
    diagnose=True,
)


async def shutdown(signame):
    """Handle shutdown signals"""
    logger.info(f"Received {signame}, shutting down...")
    if 'manager' in globals():
        manager.stop_all_nodes()
    loop = asyncio.get_event_loop()
    loop.stop()


async def main():

    manager.boot()
    try:
        # 重定向MCP服务器输出到日志文件，避免与CLI输出冲突
        import subprocess
        import sys

        # 创建日志文件
        log_file = os.path.expanduser("~/.robonix_mcp.log")

        # 启动MCP服务器进程，重定向输出到日志文件
        mcp_process = subprocess.Popen(
            [sys.executable, "-c",
             "import asyncio; import sys; import os; sys.path.append(os.path.dirname(os.path.abspath('.'))); from manager.eaios_decorators import mcp_start; asyncio.run(mcp_start())"],
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
    parser.add_argument("--config", type=str, required=True,
                        help="Path to the configuration file")
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
