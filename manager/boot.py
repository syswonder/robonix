import process_manage

def cmdline(manager: process_manage.CapabilityManager):
    """Entry point for command line interface."""
    while True:
        print(">", end="")
        cmd = input().strip().split()
        if len(cmd) == 0:
            continue

        if cmd[0] == "list":
            manager.print_available_capabilities()
        elif cmd[0] == "start":
            if len(cmd) < 2:
                print("Error: Missing capability name.")
            capability_name = cmd[1]
            manager.start_capability(capability_name)
        elif cmd[0] == "stop":
            if len(cmd) < 2:
                print("Error: Missing capability name.")
            capability_name = cmd[1]
            manager.stop_capability(capability_name)
        elif cmd[0] == "output":
            if len(cmd) < 2:
                print("Error: Missing capability name.")
            capability_name = cmd[1]
            manager.print_capability_output(capability_name)
        elif cmd[0] == "pids":
            pids = manager.get_all_running_PIDs()
            print(f"Running PIDs: {pids}")
        elif cmd[0] == "exit":
            manager.stop_all_capabilities()
            break
        else:
            print(f"Error: Unknown command: {cmd}")

if __name__ == '__main__':
    manager = process_manage.CapabilityManager("./")
    manager.boot()

    cmdline(manager)



