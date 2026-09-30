"""Profile editor, API settings and the confirmation prompt for risky commands."""

import json
from dataclasses import replace

from PySide6.QtWidgets import (QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
                               QFormLayout, QLabel, QLineEdit, QMessageBox, QPlainTextEdit,
                               QSpinBox, QVBoxLayout)

from ..api_client import DEFAULT_API_URL
from ..profiles import TEMPLATES, Profile
from ..secrets import API_TOKEN_ENV

SOURCE_TEXT = {
    "env": "found in the environment variable",
    "keyring": "found in the system keyring",
    "memory": "typed earlier in this session (memory only)",
    "none": "no key found (fine for local servers)",
}


class ProfileDialog(QDialog):
    """Edits one profile. The key field is optional and never saved to the profiles file."""

    def __init__(self, profile=None, resolved_source="none", keyring_available=False, parent=None):
        super().__init__(parent)
        self.setWindowTitle("LLM profile")
        self.setMinimumWidth(480)
        self._profile = profile
        layout = QVBoxLayout(self)
        form = QFormLayout()
        layout.addLayout(form)

        if profile is None:  # a new profile can start from an example
            self.template = QComboBox()
            self.template.addItems(["Custom", *TEMPLATES])
            self.template.currentTextChanged.connect(self._apply_template)
            form.addRow("Start from", self.template)

        self.name = QLineEdit()
        self.base_url = QLineEdit()
        self.base_url.setPlaceholderText("http://localhost:11434/v1")
        self.model = QLineEdit()
        self.api_key_env = QLineEdit()
        self.api_key_env.setPlaceholderText("e.g. DEEPSEEK_API_KEY (empty for local servers)")
        self.api_key = QLineEdit()
        self.api_key.setEchoMode(QLineEdit.EchoMode.Password)
        self.api_key.setPlaceholderText("optional: kept in memory only")
        self.remember = QCheckBox("Remember in the system keyring")
        self.remember.setEnabled(keyring_available)
        if not keyring_available:
            self.remember.setToolTip("Install the optional 'keyring' package to enable this.")
        self.key_source = QLabel(SOURCE_TEXT.get(resolved_source, ""))
        self.vision = QCheckBox("The model can see images (send screenshots to it)")
        self.simplify = QCheckBox("Simplify tool schemas (for servers that reject oneOf/minItems)")
        self.stream = QCheckBox("Stream the answer")
        self.temperature_on = QCheckBox("Set temperature")
        self.temperature = QDoubleSpinBox()
        self.temperature.setRange(0.0, 2.0)
        self.temperature.setSingleStep(0.1)
        self.temperature.setEnabled(False)
        self.temperature_on.toggled.connect(self.temperature.setEnabled)
        self.max_steps = QSpinBox()
        self.max_steps.setRange(1, 500)
        self.timeout = QSpinBox()
        self.timeout.setRange(5, 3600)
        self.timeout.setSuffix(" s")
        self.context_chars = QSpinBox()
        self.context_chars.setRange(5_000, 2_000_000)
        self.context_chars.setSingleStep(10_000)
        self.extra_prompt = QPlainTextEdit()
        self.extra_prompt.setFixedHeight(70)
        self.extra_prompt.setPlaceholderText("Optional text added to the built-in system prompt")

        form.addRow("Name", self.name)
        form.addRow("Base URL", self.base_url)
        form.addRow("Model", self.model)
        form.addRow("Key variable", self.api_key_env)
        form.addRow("API key", self.api_key)
        form.addRow("", self.remember)
        form.addRow("", self.key_source)
        form.addRow("", self.vision)
        form.addRow("", self.simplify)
        form.addRow("", self.stream)
        form.addRow(self.temperature_on, self.temperature)
        form.addRow("Max steps per request", self.max_steps)
        form.addRow("Idle timeout", self.timeout)
        form.addRow("History size (chars)", self.context_chars)
        form.addRow("Extra prompt", self.extra_prompt)

        self.error = QLabel()
        self.error.setStyleSheet("color: #d9534f;")
        self.error.setWordWrap(True)
        layout.addWidget(self.error)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self._accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self._load(profile or Profile(name="", base_url="", model=""))

    def _load(self, profile):
        self.name.setText(profile.name)
        self.base_url.setText(profile.base_url)
        self.model.setText(profile.model)
        self.api_key_env.setText(profile.api_key_env)
        self.vision.setChecked(profile.vision)
        self.simplify.setChecked(profile.simplify_schema)
        self.stream.setChecked(profile.stream)
        self.temperature_on.setChecked(profile.temperature is not None)
        self.temperature.setValue(profile.temperature if profile.temperature is not None else 0.2)
        self.max_steps.setValue(profile.max_steps)
        self.timeout.setValue(int(profile.request_timeout))
        self.context_chars.setValue(profile.context_chars)
        self.extra_prompt.setPlainText(profile.system_prompt_extra)

    def _apply_template(self, title):
        if title in TEMPLATES:
            self._load(TEMPLATES[title])

    def profile(self):
        """The profile as edited (other fields of the original are kept)."""
        base = self._profile or Profile(name="", base_url="", model="")
        return replace(
            base,
            name=self.name.text().strip(),
            base_url=self.base_url.text().strip(),
            model=self.model.text().strip(),
            api_key_env=self.api_key_env.text().strip(),
            vision=self.vision.isChecked(),
            simplify_schema=self.simplify.isChecked(),
            stream=self.stream.isChecked(),
            temperature=self.temperature.value() if self.temperature_on.isChecked() else None,
            max_steps=self.max_steps.value(),
            request_timeout=float(self.timeout.value()),
            context_chars=self.context_chars.value(),
            system_prompt_extra=self.extra_prompt.toPlainText().strip(),
        )

    def typed_key(self):
        return self.api_key.text()

    def remember_in_keyring(self):
        return self.remember.isChecked() and bool(self.api_key.text())

    def _accept(self):
        problems = self.profile().problems()
        if problems:
            self.error.setText("\n".join(problems))
            return
        self.accept()


class ApiSettingsDialog(QDialog):
    """Where the editor's API is, its token, and whether risky commands need confirmation."""

    def __init__(self, api_url, confirm_risky, token_source="none", parent=None):
        super().__init__(parent)
        self.setWindowTitle("Editor API settings")
        self.setMinimumWidth(440)
        layout = QVBoxLayout(self)
        form = QFormLayout()
        layout.addLayout(form)
        self.api_url = QLineEdit(api_url or DEFAULT_API_URL)
        self.token = QLineEdit()
        self.token.setEchoMode(QLineEdit.EchoMode.Password)
        self.token.setPlaceholderText(f"optional: kept in memory only (or set {API_TOKEN_ENV})")
        self.confirm_risky = QCheckBox("Ask before save, export, load and new-document commands")
        self.confirm_risky.setChecked(confirm_risky)
        form.addRow("API URL", self.api_url)
        form.addRow("Token", self.token)
        form.addRow("", QLabel(SOURCE_TEXT.get(token_source, "")))
        form.addRow("", self.confirm_risky)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)


def ask_confirmation(parent, command, arguments):
    """Modal prompt before a command that writes files or discards work; True = allow."""
    box = QMessageBox(parent)
    box.setObjectName("confirm_dialog")
    box.setIcon(QMessageBox.Icon.Warning)
    box.setWindowTitle("Confirm command")
    box.setText(f"The assistant wants to run '{command}'.")
    box.setInformativeText("This can write files or discard the current model.")
    box.setDetailedText(json.dumps(arguments, indent=2))
    allow = box.addButton("Allow", QMessageBox.ButtonRole.AcceptRole)
    deny = box.addButton("Deny", QMessageBox.ButtonRole.RejectRole)
    box.setDefaultButton(deny)
    box.setEscapeButton(deny)
    box.exec()
    return box.clickedButton() is allow
