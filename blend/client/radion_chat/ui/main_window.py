"""The main window: chat on the left, screenshots on the right, all network work on threads."""

from PySide6.QtCore import Qt, QThread
from PySide6.QtWidgets import (QComboBox, QFileDialog, QHBoxLayout, QLabel, QMainWindow, QMessageBox,
                               QPushButton, QSplitter, QVBoxLayout, QWidget)

from ..api_client import DEFAULT_API_URL
from ..conversation import default_file_name, save_conversation
from ..profiles import ProfileStore
from ..secrets import API_TOKEN_ACCOUNT, API_TOKEN_ENV, SecretStore, llm_key_account
from .chat_input import ChatInput
from .chat_view import ChatView
from .dialogs import ApiSettingsDialog, ProfileDialog, ask_confirmation
from .image_panel import ScreenshotPanel
from .worker import AgentWorker, HealthPoller

SHUTDOWN_WAIT_MS = 5000
VISION_OFF_NOTICE = ("This profile has vision turned off: screenshots are shown here "
                     "but not sent to the model. Enable it in the profile if the model can see images.")


class MainWindow(QMainWindow):
    def __init__(self, store=None, secrets=None, poll_interval_ms=2000, confirm=ask_confirmation):
        super().__init__()
        self.setWindowTitle("Radion Chat")
        self.resize(1200, 760)
        if store is None:
            store = ProfileStore()
            store.load()
        self.store = store
        self.secrets = secrets or SecretStore()
        self._confirm = confirm
        self.busy = False
        self._vision_warned = False
        self._undoable = 0  # editor undo steps the latest request added

        self._build_ui()
        self._start_threads(poll_interval_ms)
        self._populate_profiles()
        if not self.store.profiles:
            self.chat.add_notice("No LLM profile yet: click 'New' to create one "
                                 "(examples for Ollama and DeepSeek are offered).")
        self._refresh_buttons()

    # -- construction ------------------------------------------------------------------

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)

        bar = QHBoxLayout()
        outer.addLayout(bar)
        bar.addWidget(QLabel("Profile:"))
        self.profile_combo = QComboBox()
        self.profile_combo.setMinimumWidth(160)
        self.profile_combo.currentTextChanged.connect(self._profile_selected)
        bar.addWidget(self.profile_combo)
        self.edit_button = self._button("Edit", self._edit_profile, bar)
        self.new_button = self._button("New", self._new_profile, bar)
        self.delete_button = self._button("Delete", self._delete_profile, bar)
        self.api_button = self._button("API settings...", self._api_settings, bar)
        bar.addStretch(1)
        self.undo_button = self._button("Undo this request", self._undo_request, bar)
        self.new_chat_button = self._button("New chat", self._new_chat, bar)
        self.save_button = self._button("Save conversation...", self._save_conversation, bar)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        outer.addWidget(splitter, 1)
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        self.chat = ChatView()
        left_layout.addWidget(self.chat, 1)
        row = QHBoxLayout()
        left_layout.addLayout(row)
        self.input = ChatInput()
        self.input.submitted.connect(self._submit_from_input)
        row.addWidget(self.input, 1)
        self.send_button = QPushButton("Send")
        self.send_button.setFixedHeight(80)
        self.send_button.clicked.connect(self._send_or_stop)
        row.addWidget(self.send_button)
        splitter.addWidget(left)
        self.panel = ScreenshotPanel()
        self.panel.setMinimumWidth(300)
        splitter.addWidget(self.panel)
        splitter.setSizes([740, 460])

        self.api_label = QLabel("Editor API: checking...")
        self.run_label = QLabel("Ready")
        self.statusBar().addWidget(self.api_label, 1)
        self.statusBar().addPermanentWidget(self.run_label)

    def _button(self, text, slot, layout):
        button = QPushButton(text)
        button.clicked.connect(slot)
        layout.addWidget(button)
        return button

    def _start_threads(self, poll_interval_ms):
        self.worker = AgentWorker()
        self._agent_thread = QThread(self)
        self.worker.moveToThread(self._agent_thread)
        worker = self.worker
        worker.step.connect(self._on_step)
        worker.text_delta.connect(self.chat.append_assistant)
        worker.tool_call.connect(self.chat.add_tool_call)
        worker.tool_result.connect(self._on_tool_result)
        worker.error.connect(lambda message: self.chat.add_notice(message, error=True))
        worker.finished.connect(self._on_finished)
        worker.undone.connect(self._on_undone)
        worker.confirm_requested.connect(self._on_confirm_requested)
        self._agent_thread.start()

        self.poller = HealthPoller(poll_interval_ms / 1000)
        self.poller.state.connect(self._on_api_state)
        self.poller.start()

    # -- profiles ----------------------------------------------------------------------

    def _populate_profiles(self):
        self.profile_combo.blockSignals(True)
        self.profile_combo.clear()
        self.profile_combo.addItems(list(self.store.profiles))
        if self.store.active:
            self.profile_combo.setCurrentText(self.store.active)
        self.profile_combo.blockSignals(False)
        self.poller.set_target(*self._api_target())

    def _profile_selected(self, name):
        if name and name != self.store.active:
            self.store.active = name
            self.store.save()
            self.poller.set_target(*self._api_target())

    def _edit_profile(self):
        profile = self.store.current()
        if profile:
            self._run_profile_dialog(profile)

    def _new_profile(self):
        self._run_profile_dialog(None)

    def _run_profile_dialog(self, profile):
        account = llm_key_account(profile.name) if profile else ""
        source = self.secrets.resolve(account, profile.api_key_env).source if profile else "none"
        dialog = ProfileDialog(profile, source, self.secrets.keyring_available(), self)
        if not dialog.exec():
            return
        edited = dialog.profile()
        new_account = llm_key_account(edited.name)
        if dialog.typed_key():
            self.secrets.set_memory(new_account, dialog.typed_key())
            if dialog.remember_in_keyring() and not self.secrets.save_to_keyring(new_account, dialog.typed_key()):
                self.statusBar().showMessage("Could not save the key in the system keyring.", 6000)
        self.store.put(edited, replacing=profile.name if profile else None)
        self.store.active = edited.name
        self.store.save()
        self._populate_profiles()

    def _delete_profile(self):
        profile = self.store.current()
        if profile and QMessageBox.question(self, "Delete profile", f"Delete profile '{profile.name}'?") \
                == QMessageBox.StandardButton.Yes:
            self.store.delete(profile.name)
            self.store.save()
            self._populate_profiles()
            self._refresh_buttons()

    def _api_settings(self):
        profile = self.store.current()
        url = profile.api_url if profile else DEFAULT_API_URL
        token_source = self.secrets.resolve(API_TOKEN_ACCOUNT, API_TOKEN_ENV).source
        dialog = ApiSettingsDialog(url, self.store.confirm_risky, token_source, self)
        if not dialog.exec():
            return
        if profile:
            profile.api_url = dialog.api_url.text().strip() or DEFAULT_API_URL
        if dialog.token.text():
            self.secrets.set_memory(API_TOKEN_ACCOUNT, dialog.token.text())
        self.store.confirm_risky = dialog.confirm_risky.isChecked()
        self.store.save()
        self.poller.set_target(*self._api_target())

    def _api_target(self):
        profile = self.store.current()
        token = self.secrets.resolve(API_TOKEN_ACCOUNT, API_TOKEN_ENV).value or None
        return (profile.api_url if profile else DEFAULT_API_URL), token

    # -- sending -----------------------------------------------------------------------

    def _submit_from_input(self):
        if not self.busy:
            self._send()

    def _send_or_stop(self):
        if self.busy:
            self.worker.cancel()
            self.run_label.setText("Stopping...")
        else:
            self._send()

    def _send(self):
        text = self.input.toPlainText().strip()
        if not text:
            return
        profile = self.store.current()
        if profile is None:
            self.chat.add_notice("Create an LLM profile first (New).", error=True)
            return
        problems = profile.problems()
        if problems:
            self.chat.add_notice("The profile is incomplete: " + " ".join(problems), error=True)
            return
        key = self.secrets.resolve(llm_key_account(profile.name), profile.api_key_env).value
        token = self._api_target()[1]
        self.worker.configure(profile, key, token, self.store.confirm_risky)
        self.chat.add_user(text)
        self.input.clear()
        self._set_busy(True)
        self.worker.submit(text)

    def _set_busy(self, busy):
        self.busy = busy
        self.send_button.setText("Stop" if busy else "Send")
        self.run_label.setText("Working..." if busy else "Ready")
        self._refresh_buttons()

    def _refresh_buttons(self):
        idle = not self.busy
        has_profile = self.store.current() is not None
        for button in (self.profile_combo, self.new_button, self.api_button, self.new_chat_button,
                       self.save_button):
            button.setEnabled(idle)
        self.edit_button.setEnabled(idle and has_profile)
        self.delete_button.setEnabled(idle and has_profile)
        self.undo_button.setEnabled(idle and self._undoable > 0)

    # -- worker signals ----------------------------------------------------------------

    def _on_step(self, step, max_steps):
        self.chat.end_assistant()
        self.run_label.setText(f"Step {step}/{max_steps}")

    def _on_tool_result(self, call_id, name, summary, is_error, image_png, text):
        self.chat.finish_tool(call_id, summary, is_error, text)
        if image_png and self.panel.add_png(image_png):
            profile = self.store.current()
            if profile and not profile.vision and not self._vision_warned:
                self._vision_warned = True
                self.chat.add_notice(VISION_OFF_NOTICE)

    def _on_finished(self, reason, undoable_steps):
        self._undoable = undoable_steps
        if reason == "cancelled":
            self.chat.add_notice("Stopped.")
        self._set_busy(False)

    def _on_undone(self, steps):
        self._undoable = 0
        if steps:
            self.chat.add_notice(f"Undid {steps} edit(s) of the last request.")
        self._refresh_buttons()

    def _on_confirm_requested(self, command, arguments):
        self.worker.answer_confirm(self._confirm(self, command, arguments))

    def _on_api_state(self, reachable, text):
        mark = "●" if reachable else "○"
        self.api_label.setText(f"{mark} {text}")

    # -- toolbar actions ---------------------------------------------------------------

    def _undo_request(self):
        self.undo_button.setEnabled(False)
        self.worker.request_undo()

    def _new_chat(self):
        self.worker.request_reset()
        self.chat.clear()
        self.panel.clear()
        self._undoable = 0
        self._vision_warned = False
        self._refresh_buttons()

    def _save_conversation(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save conversation", default_file_name(),
                                              "JSON (*.json)")
        if path:
            profile = self.store.current()
            save_conversation(path, profile.name if profile else "", self.worker.conversation())

    # -- shutdown ----------------------------------------------------------------------

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)

    def shutdown(self):
        """Stops a request in flight and both threads; safe to call twice."""
        self.worker.cancel()  # shuts the LLM socket down, so the agent thread returns quickly
        self.poller.stop()
        self._agent_thread.quit()
        for thread in (self._agent_thread, self.poller):
            if not thread.wait(SHUTDOWN_WAIT_MS):
                # Last resort: an editor call that never returns must not keep the app alive.
                thread.terminate()
                thread.wait()
