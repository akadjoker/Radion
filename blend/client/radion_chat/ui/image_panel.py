
from PySide6.QtCore import QSize, Qt, Signal
from PySide6.QtGui import QIcon, QPixmap
from PySide6.QtWidgets import (QDialog, QHBoxLayout, QLabel, QSizePolicy,
                               QToolButton, QVBoxLayout, QWidget)

MAX_HISTORY = 8
THUMB_SIZE = QSize(72, 54)
PLACEHOLDER = "No screenshot yet.\nThe model's screenshots appear here."


class ScaledLabel(QLabel):
    clicked = Signal()

    def __init__(self):
        super().__init__()
        self._source = None
        self.setMinimumSize(1, 1)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)

    def set_source(self, pixmap):
        self._source = pixmap
        self._rescale()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._rescale()

    def mouseReleaseEvent(self, event):
        if self._source is not None:
            self.clicked.emit()
        super().mouseReleaseEvent(event)

    def _rescale(self):
        if self._source is not None:
            self.setPixmap(self._source.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                               Qt.TransformationMode.SmoothTransformation))


class ImageDialog(QDialog):
    def __init__(self, pixmap, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Screenshot {pixmap.width()}x{pixmap.height()}")
        layout = QVBoxLayout(self)
        self.label = ScaledLabel()
        self.label.set_source(pixmap)
        layout.addWidget(self.label)
        self.resize(min(pixmap.width(), 1100), min(pixmap.height(), 800))


class ScreenshotPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.images = []
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.main = ScaledLabel()
        self.main.setText(PLACEHOLDER)
        self.main.setStyleSheet("QLabel { border: 1px solid palette(mid); }")
        self.main.clicked.connect(lambda: self.enlarge(len(self.images) - 1))
        self.main.setCursor(Qt.CursorShape.PointingHandCursor)
        layout.addWidget(self.main, 1)

        self._strip = QHBoxLayout()
        self._strip.addStretch(1)
        layout.addLayout(self._strip)
        self.thumbnails = []
        self.dialog = None

    def add_png(self, data):
        pixmap = QPixmap()
        if not pixmap.loadFromData(data):
            return False
        self.images.append(pixmap)
        self.main.set_source(pixmap)
        self._rebuild_strip()
        return True

    def enlarge(self, index):
        if 0 <= index < len(self.images):
            self.dialog = ImageDialog(self.images[index], self)
            self.dialog.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
            self.dialog.show()

    def clear(self):
        self.images.clear()
        self.main.set_source(None)
        self.main.setPixmap(QPixmap())
        self.main.setText(PLACEHOLDER)
        self._rebuild_strip()

    def _rebuild_strip(self):
        del self.images[:-MAX_HISTORY]
        for button in self.thumbnails:
            button.setParent(None)
            button.deleteLater()
        self.thumbnails = []
        for index, pixmap in enumerate(self.images):
            button = QToolButton()
            button.setIcon(QIcon(pixmap.scaled(THUMB_SIZE, Qt.AspectRatioMode.KeepAspectRatio,
                                               Qt.TransformationMode.SmoothTransformation)))
            button.setIconSize(THUMB_SIZE)
            button.setToolTip("Click to enlarge")
            button.clicked.connect(lambda _checked=False, i=index: self.enlarge(i))
            self._strip.insertWidget(self._strip.count() - 1, button)
            self.thumbnails.append(button)
