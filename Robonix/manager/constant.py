import os

BASE_PATH = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
BASE_SKILL_PATH = os.path.join(BASE_PATH, "skill")
# Keep both files for code migration: internal skill/__init__.py and external __export__.py
INIT_FILE = os.path.join(BASE_SKILL_PATH, "__init__.py")
EXPORT_FILE = os.path.join(os.path.dirname(BASE_PATH), "__export__.py")
BASE_CAP_PATH = os.path.join(BASE_PATH, "capability")
