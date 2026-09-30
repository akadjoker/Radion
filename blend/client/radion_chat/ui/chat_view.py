"""The conversation transcript: bubbles, streaming text and collapsible tool-call entries."""

import json

from PySide6.QtCore import QSize, Qt
from PySide6.QtWidgets import (QFrame, QHBoxLayout, QLabel, QPlainTextEdit, QScrollArea,
                               QSizePolicy, QToolButton, QVBoxLayout, QWidget)

ERROR_COLOUR = "#d9534f"


class Bubble(QLabel):
    def __init__(self, text, role):
        super().__init__()
        self.role = role
        self.setWordWrap(True)
        self.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.setTextFormat(Qt.TextFormat.PlainText if role == "user" else Qt.TextFormat.MarkdownText)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum)
        background = "palette(highlight)" if role == "user" else "palette(alternate-base)"
        colour = "palette(highlighted-text)" if role == "user" else "palette(text)"
        self.setStyleSheet(f"QLabel {{ background: {background}; color: {colour}; "
                           "border-radius: 8px; padding: 8px; }")
        self.text_so_far = ""
        self.append(text)

    def append(self, text):
        self.text_so_far += text
        self.setText(self.text_so_far)


class ToolEntry(QFrame):
    """One tool call: header with name and result summary; click to see the JSON."""

    def __init__(self, name, arguments):
        super().__init__()
        self.name = name
        self._arguments = arguments
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)

        self.toggle = QToolButton()
        self.toggle.setCheckable(True)
        self.toggle.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.toggle.setArrowType(Qt.ArrowType.RightArrow)
        self.toggle.setText(name)
        self.toggle.setStyleSheet("QToolButton { border: none; font-weight: bold; }")
        self.toggle.toggled.connect(self._toggled)

        self.status = QLabel("running...")
        self.status.setWordWrap(True)
        self.status.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
        self.status.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)

        header = QHBoxLayout()
        header.addWidget(self.toggle, 0, Qt.AlignmentFlag.AlignTop)
        header.addWidget(self.status, 1, Qt.AlignmentFlag.AlignTop)
        layout.addLayout(header)

        self.details = QPlainTextEdit()
        self.details.setReadOnly(True)
        self.details.setFixedHeight(160)
        self.details.setVisible(False)
        self.details.setPlainText("Arguments:\n" + _pretty(arguments))
        layout.addWidget(self.details)
        self.is_error = False

    def finish(self, summary, is_error, text):
        self.is_error = is_error
        # Compact JSON has no spaces: a zero-width space after commas lets the label wrap.
        self.status.setText(("error: " if is_error else "") + summary.replace(",", ",\u200b"))
        if is_error:
            self.status.setStyleSheet(f"color: {ERROR_COLOUR};")
        self.details.setPlainText(f"Arguments:\n{_pretty(self._arguments)}\n\nResult:\n{text}")

    def _toggled(self, expanded):
        self.toggle.setArrowType(Qt.ArrowType.DownArrow if expanded else Qt.ArrowType.RightArrow)
        self.details.setVisible(expanded)


def _pretty(arguments):
    if isinstance(arguments, str):  # malformed arguments arrive as the model's raw text
        return arguments
    return json.dumps(arguments, indent=2)


class _Transcript(QWidget):
    """The scrolled content. Its own minimum size hint is 0: a layout of word-wrapped labels
    reports the height it would need at the *narrowest* width, which left a tall blank area
    under the last message. The scroll area then sizes it with height-for-width instead."""

    def minimumSizeHint(self):
        return QSize(0, 0)


class ChatView(QScrollArea):
    def __init__(self):
        super().__init__()
        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setFrameShape(QFrame.Shape.NoFrame)
        body = _Transcript()
        self._layout = QVBoxLayout(body)
        self._layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.setWidget(body)
        self.items = []             # every widget in the transcript, in order
        self._rows = []             # the widgets that actually sit in the layout
        self.tool_entries = {}      # tool call id -> ToolEntry
        self._assistant = None      # the bubble currently receiving streamed text
        self._at_bottom = True
        bar = self.verticalScrollBar()
        bar.valueChanged.connect(lambda value: setattr(self, "_at_bottom", value >= bar.maximum() - 4))
        bar.rangeChanged.connect(lambda _low, high: self._at_bottom and bar.setValue(high))

    def add_user(self, text):
        self.end_assistant()
        self._add(Bubble(text, "user"), side="right")

    def append_assistant(self, text):
        if self._assistant is None:
            self._assistant = Bubble("", "assistant")
            self._add(self._assistant, side="left")
        self._assistant.append(text)

    def end_assistant(self):
        """The next streamed text starts a new bubble (after a tool call, say)."""
        self._assistant = None

    def add_tool_call(self, call_id, name, arguments):
        self.end_assistant()
        entry = ToolEntry(name, arguments)
        self.tool_entries[call_id] = entry
        self._add(entry)

    def finish_tool(self, call_id, summary, is_error, text):
        entry = self.tool_entries.get(call_id)
        if entry:
            entry.finish(summary, is_error, text)

    def add_notice(self, text, error=False):
        self.end_assistant()
        label = QLabel(text)
        label.setWordWrap(True)
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        label.setStyleSheet(f"color: {ERROR_COLOUR};" if error else "color: palette(mid);")
        self._add(label)

    def clear(self):
        for row in self._rows:
            row.setParent(None)
            row.deleteLater()
        self._rows.clear()
        self.items.clear()
        self.tool_entries.clear()
        self._assistant = None

    def _add(self, widget, side=None):
        """Adds a widget as a new row. Bubbles take 80% of the width, pushed to `side`."""
        self.items.append(widget)
        if side is None:
            row = widget
        else:
            row = QWidget()
            layout = QHBoxLayout(row)
            layout.setContentsMargins(0, 0, 0, 0)
            gap = QWidget()
            layout.addWidget(gap, 1)
            layout.insertWidget(0 if side == "left" else 1, widget, 4)
        self._rows.append(row)
        self._layout.addWidget(row)
        return widget
