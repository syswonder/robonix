from robonix.manager.eaios_decorators import eaios

@eaios.plugin("navigation2","simple_navigation")
def nv_test():
    print("nv test simple_navigation")
    return "nv test simple_navigation"

@eaios.plugin("navigation2","simple_navigation")
def set_goal():
    print("nv test simple_navigation set_goal")
    return "nv test simple_navigation set_goal"