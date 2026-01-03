import sys
import os
from robonix.manager.eaios_decorators import eaios, package_init,FunctionRegistry,mcp_start

import yaml
package_init("config/include/ranger_test.yml")
with open("test.txt","w") as f:
    import time
    f.write(str(time.time()))
    registry = FunctionRegistry()
    print(f"[eaios] Finalized with {registry.gen_lens()} functions registered.")
    mcp_start()