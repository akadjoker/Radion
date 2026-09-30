from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QPlainTextEdit


class ChatInput(QPlainTextEdit):
    submitted = Signal()

    def __init__(self):
        super().__init__()
        self.setPlaceholderText("Describe what to build, e.g. \"make a tank with a rotating turret\"  "
                                "(Enter sends, Shift+Enter for a new line)")
        self.setFixedHeight(80)
        self.setTabChangesFocus(True)

    def keyPressEvent(self, event):
        is_enter = event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter)
        if is_enter and not event.modifiers() & Qt.KeyboardModifier.ShiftModifier:
            self.submitted.emit()
            return
        super().keyPressEvent(event)
