# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
import sys

# Add the src directory to the Python path so Sphinx can import our modules
sys.path.insert(0, os.path.abspath('../src'))

project = 'ROS2 RobotFramework'
copyright = '2025, ROS2 Robot Framework Team'
author = 'ROS2 Robot Framework Team'
release = '1.0.2'
version = '1.0.2'

# The master toctree document
master_doc = 'index'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.viewcode',
    'sphinx.ext.napoleon',
    'sphinx.ext.intersphinx',
    'myst_parser',
    'sphinx_autodoc_typehints',
    'sphinx_copybutton',
    'sphinx_design',
]

# MyST Parser configuration
myst_enable_extensions = [
    'colon_fence',
    'deflist',
    'dollarmath',
    'fieldlist',
    'html_admonition',
    'html_image',
    'linkify',
    'replacements',
    'smartquotes',
    'strikethrough',
    'substitution',
    'tasklist',
]

myst_url_schemes = ['http', 'https', 'ftp', 'mailto']

# Autodoc configuration
autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'special-members': '__init__',
    'undoc-members': True,
    'exclude-members': '__weakref__'
}

autodoc_typehints = 'description'
autoclass_content = 'both'

# Autosummary configuration
autosummary_generate = True

# Intersphinx configuration
intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'robotframework': ('https://robotframework.org/robotframework/latest/libraries/', None),
    'ros2': ('https://docs.ros.org/en/rolling/', None),
}

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_css_files = ['css/custom.css']

# Theme options
html_theme_options = {
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'collapse_navigation': True,
    'sticky_navigation': True,
    'navigation_depth': 3,
    'includehidden': False,
    'titles_only': False
}

# -- Options for HTMLHelp output ---------------------------------------------
htmlhelp_basename = 'ROS2RobotFrameworkdoc'

# -- Options for LaTeX output ------------------------------------------------
latex_elements = {
    'papersize': 'letterpaper',
    'pointsize': '10pt',
    'figure_align': 'htbp',
}

latex_documents = [
    (master_doc, 'ROS2RobotFramework.tex', 'ROS2 RobotFramework Documentation',
     'ROS2 Robot Framework Team', 'manual'),
]

# -- Options for manual page output ------------------------------------------
man_pages = [
    (master_doc, 'ros2-robotframework', 'ROS2 RobotFramework Documentation',
     [author], 1)
]

# -- Options for Texinfo output ----------------------------------------------
texinfo_documents = [
    (master_doc, 'ROS2RobotFramework', 'ROS2 RobotFramework Documentation',
     author, 'ROS2RobotFramework', 'One line description of project.',
     'Miscellaneous'),
]

# -- Options for epub output -------------------------------------------------
epub_title = project
epub_author = author
epub_publisher = author
epub_copyright = copyright
epub_exclude_files = ['search.html']


