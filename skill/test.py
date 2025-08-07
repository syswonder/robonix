import os
import sys

root_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if root_dir not in sys.path:
    sys.path.append(root_dir)
print("[DEBUG] skill root_dir:", root_dir)
from DeepEmbody.manager.eaios_decorators import package_init, mcp_start,eaios

package_init("config/include/ranger_test.yml")
print(eaios.FUNCTION_REGISTRY)

@eaios.caller
def do_test():
    print("[DEBUG] skill root_dir:", root_dir)
    test_nv()

do_test()