from DeepEmbody.manager.eaios_decorators import eaios

@eaios.plugin("navigation2","ros2_navigation")
def nv_test():
    print("nv test ros2_navigation")
    return "nv test ros2_navigation"
    