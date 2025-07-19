import os
import sys
from constant import BASE_SKILL_PATH, INIT_FILE, BASE_PATH

if os.path.dirname(BASE_PATH) not in sys.path:
    sys.path.append(os.path.dirname(BASE_PATH))
from DeepEmbody.manager.eaios_decorators import package_init, mcp_start,eaios
import DeepEmbody.skill 

package_init("config/include.yaml")
print(eaios.FUNCTION_REGISTRY)

@eaios.caller
def do_test():
    return test_nv()

res = do_test()
print("res",res)