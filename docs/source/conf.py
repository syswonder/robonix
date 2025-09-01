# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'DeepEmbody OS'
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
    'robot_control_pb2'
]

# Napoleon settings
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = False
napoleon_include_private_with_doc = False

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
