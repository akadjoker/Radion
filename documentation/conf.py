# Radion documentation configuration.
#
# Build with, from the repo root:
#   sphinx-build -b html documentation documentation/_build/html

project = "Radion"
author = "akadjoker"
copyright = "2026, akadjoker"
release = "0.1.0"

extensions = []

templates_path = []
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

root_doc = "index"

html_theme = "furo"
html_title = "Radion"
html_theme_options = {
    "navigation_with_keys": True,
}
