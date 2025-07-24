import process_manage
import node
from log import logger
from cmdline import CLI
import depend
from eaios_decorators import package_init

if __name__ == "__main__":
    node_list = node.get_node_details("config/include.yaml")
    depend.check_depend("config/include.yaml")
    manager = process_manage.RuntimeManager(node_list)
    package_init("config/include.yaml")
    try:
        manager.boot()
        cli = CLI(manager)
        cli.run()

    except KeyboardInterrupt:
        logger.info("Exiting...")
    finally:
        manager.stop_all_nodes()
