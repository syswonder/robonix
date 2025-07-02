import process_manage
import node
import sys
from cmdline import CLI

if __name__ == "__main__":
    manager = process_manage.RuntimeManager(
        node.get_node_details("./driver/") + node.get_node_details("./capability/")
    )

    try:
        manager.boot()

        cli = CLI(manager)
        cli.run()

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        manager.stop_all_nodes()
