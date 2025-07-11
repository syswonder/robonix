import process_manage
import node
import sys
from loguru import logger
from cmdline import CLI

logger.remove()
logger.add(
    sys.stderr,
    format="[{elapsed} {name}] {message}",
    level="INFO",
    colorize=True,
    backtrace=True,
    diagnose=True,
)

if __name__ == "__main__":

    manager = process_manage.RuntimeManager(
        node.get_node_details("./driver/") + node.get_node_details("./capability/")
    )

    try:
        manager.boot()

        cli = CLI(manager)
        cli.run()

    except KeyboardInterrupt:
        logger.info("Exiting...")
    finally:
        manager.stop_all_nodes()
