import sys
import os

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_ROOT)

from robonix.simulator.genesis.robot1 import main

if __name__ == "__main__":
    main()
