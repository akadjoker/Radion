import sys

from PySide6.QtWidgets import QApplication

from .main_window import MainWindow


def main(argv=None):
    app = QApplication(argv if argv is not None else sys.argv)
    app.setApplicationName("radion_chat")
    window = MainWindow()
    window.show()
    return app.exec()
