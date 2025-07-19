import os
import sys

root_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if root_dir not in sys.path:
    sys.path.append(root_dir)
from DeepEmbody.manager.eaios_decorators import package_init, mcp_start,eaios

package_init("config/include.yaml")
print(eaios.FUNCTION_REGISTRY)

@eaios.caller
def do_test():
    test_nv()

do_test()