import os
BASE_PATH = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
BASE_SKILL_PATH = os.path.join(BASE_PATH,"skill")
INIT_FILE = os.path.join(BASE_SKILL_PATH, "__init__.py")
BASE_CAP_PATH = os.path.join(BASE_PATH,"capability")