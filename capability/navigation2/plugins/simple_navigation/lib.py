from DeepEmbody.manager.eaios_decorators import eaios

@eaios.plugin("navigation2","simple_navigation")
def nv_test():
    print("nv test simple_navigation")
    return "nv test simple_navigation"