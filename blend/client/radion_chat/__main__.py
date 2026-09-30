import sys

try:
    from .ui.app import main
except ImportError as error:
    if "PySide6" not in str(error):
        raise
    sys.exit("radion_chat needs PySide6: pip install -r requirements.txt")

raise SystemExit(main())
