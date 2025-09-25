# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'robonix OS'
copyright = '2025, Syswonder'
author = 'Syswonder'

version = '0.0.1'
release = '0.0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.napoleon',
    'myst_parser'
]

templates_path = ['_templates']
exclude_patterns = []

language = 'en_US'

# Syntax highlighting
pygments_style = 'default'
highlight_language = 'python'
highlight_options = {'stripnl': False}

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_permalinks_icon = '<span>#</span>'
html_theme = 'sphinxawesome_theme'
html_static_path = ['_static']
html_css_files = [
    'custom.css',
]

# Add project root to Python path for autodoc
import os
import sys
sys.path.insert(0, os.path.abspath('../../'))

# Mock imports for dependencies not available during doc build
autodoc_mock_imports = [
    'loguru',
    'pynput',
    'genesis',
    'grpc',
    'numpy',
    'opencv',
    'cv2',
    'scipy',
    'scipy.spatial',
    'scipy.spatial.transform',
    'robot_control_pb2_grpc',
    'robot_control_pb2',
    'ultralytics',
    'ultralytics.models',
    'ultralytics.models.YOLOE',
    'ultralytics.models.YOLO',
    'ultralytics.models.FastSAM',
    'ultralytics.models.SAM',
    'ultralytics.models.RTDETR',
    'ultralytics.models.NAS',
    'ultralytics.models.YOLOWorld',
    'ultralytics.engine',
    'ultralytics.engine.model',
    'ultralytics.engine.results',
    'ultralytics.data',
    'ultralytics.data.augment',
    'ultralytics.data.base',
    'ultralytics.data.utils',
    'ultralytics.utils',
    'ultralytics.utils.ops',
    'ultralytics.utils.metrics',
    'aioconsole',
    'rclpy',
    'rclpy.node',
    'geometry_msgs',
    'geometry_msgs.msg',
    'nav2_simple_commander',
    'nav2_simple_commander.robot_navigator',
    'sensor_msgs',
    'sensor_msgs.msg',
    'std_srvs',
    'std_srvs.srv',
    'std_msgs',
    'std_msgs.msg',
    'mcp',
    'mcp.server',
    'mcp.server.fastmcp',
    'RPi',
    'RPi.GPIO',
    'yaml',
    'threading',
    'time',
    'signal',
    'logging',
    'argparse',
    'asyncio',
    'atexit',
    'math',
    'random',
    'datetime',
    'traceback',
    'typing',
    'typing.Dict',
    'typing.Optional',
    'typing.List',
    'typing.Callable',
    'typing.Tuple',
    'typing.Union'
]

# Napoleon settings
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = False
napoleon_include_private_with_doc = False

# Autodoc settings
autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'special-members': '__init__',
    'undoc-members': True,
    'exclude-members': '__weakref__'
}

# Suppress warnings for missing imports
suppress_warnings = ['autodoc.import_object']

# Source file settings
source_suffix = {
    '.rst': None,
}

# HTML theme options for sphinxawesome_theme
html_theme_options = {
    'show_breadcrumbs': True,
    'breadcrumbs_separator': ' / ',
    'main_nav_links': {
        'Docs': 'index',
        'API': 'api/index',
    },
}
