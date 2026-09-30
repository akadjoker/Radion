import json
import os
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest  # noqa: E402
from fake_llm_server import call_reply, text_reply  # noqa: E402
from PySide6.QtCore import Qt, QTimer  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication, QFileDialog, QPushButton  # noqa: E402
from radion_chat.profiles import Profile, ProfileStore  # noqa: E402
from radion_chat.secrets import SecretStore  # noqa: E402
from radion_chat.ui.chat_view import ToolEntry  # noqa: E402
from radion_chat.ui.dialogs import ProfileDialog  # noqa: E402
from radion_chat.ui.main_window import MainWindow  # noqa: E402


@pytest.fixture(scope="session")
def app():
    return QApplication.instance() or QApplication([])


def wait_for(condition, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        QApplication.processEvents()
        if condition():
            return True
        time.sleep(0.01)
    return False


@pytest.fixture
def make_window(app, tmp_path, api_server, llm_server):
    windows = []

    def factory(confirm=None, vision=False, with_profile=True):
        store = ProfileStore(tmp_path / "profiles.json")
        if with_profile:
            store.put(Profile(name="fake", base_url=llm_server.base_url, model="m",
                              api_url=api_server.url, vision=vision))
        secrets = SecretStore(environ={}, keyring_loader=lambda: None)
        kwargs = {"confirm": confirm} if confirm else {}
        window = MainWindow(store, secrets, poll_interval_ms=100, **kwargs)
        window.show()
        windows.append(window)
        return window

    yield factory
    for window in windows:
        window.close()


def send(window, text):
    window.input.setPlainText(text)
    QTest.mouseClick(window.send_button, Qt.MouseButton.LeftButton)


def idle(window):
    return wait_for(lambda: not window.busy)


def kinds(window):
    return [type(item).__name__ for item in window.chat.items]


def test_window_builds_with_its_controls(make_window):
    window = make_window()
    assert window.send_button.text() == "Send"
    assert window.profile_combo.currentText() == "fake"
    assert not window.undo_button.isEnabled()
    assert window.edit_button.isEnabled()
    assert "No screenshot" in window.panel.main.text()


def test_status_bar_shows_the_api_state(make_window, api_server):
    window = make_window()
    assert wait_for(lambda: "connected" in window.api_label.text())
    api_server.stop()
    assert wait_for(lambda: "radion_blender --api" in window.api_label.text())


def test_without_a_profile_the_user_is_guided(make_window):
    window = make_window(with_profile=False)
    assert not window.edit_button.isEnabled()
    assert "New" in window.chat.items[0].text()
    send(window, "hello")
    assert not window.busy
    assert "profile" in window.chat.items[-1].text().lower()


def test_scripted_conversation_renders_tools_and_screenshot(make_window, api_server, llm_server):
    llm_server.script += [
        call_reply([("add_primitive", {"type": "box", "name": "hull"}), ("screenshot", {})], text="Let me build."),
        text_reply("Built a **hull**."),
    ]
    window = make_window()
    send(window, "make a tank")
    assert idle(window)

    assert kinds(window) == ["Bubble", "Bubble", "ToolEntry", "ToolEntry", "QLabel", "Bubble"]
    user, plan, add, shot, notice, answer = window.chat.items
    assert user.role == "user" and user.text_so_far == "make a tank"
    assert plan.text_so_far == "Let me build."
    assert isinstance(add, ToolEntry) and add.name == "add_primitive"
    assert "hull" in add.status.text() and not add.is_error
    assert '"type": "box"' in add.details.toPlainText()
    assert add.details.isHidden()
    add.toggle.click()
    assert not add.details.isHidden()
    assert answer.text_so_far == "Built a **hull**."

    assert len(window.panel.images) == 1 and len(window.panel.thumbnails) == 1
    assert window.panel.images[0].width() == 2
    assert window.send_button.text() == "Send" and window.input.toPlainText() == ""
    assert api_server.names() == ["hull"]
    assert "vision turned off" in notice.text()


def test_api_error_shows_up_in_the_tool_entry(make_window, llm_server):
    llm_server.script += [call_reply([("add_primitive", {"type": "teapot"})]), text_reply("oops")]
    window = make_window()
    send(window, "teapot")
    assert idle(window)
    entry = window.chat.items[1]
    assert entry.is_error and "invalid_params" in entry.status.text()


def test_undo_this_request(make_window, api_server, llm_server):
    llm_server.script += [call_reply([("add_primitive", {"type": "box", "name": "a"}),
                                      ("add_primitive", {"type": "box", "name": "b"})]), text_reply("ok")]
    window = make_window()
    send(window, "two boxes")
    assert idle(window)
    assert window.undo_button.isEnabled()
    window.undo_button.click()
    assert wait_for(lambda: "Undid 2" in window.chat.items[-1].text())
    assert api_server.names() == [] and not window.undo_button.isEnabled()


def test_stop_button_interrupts_a_stalled_model(make_window, llm_server):
    llm_server.script.append(text_reply("thinking...", stall_after_first_chunk=True))
    window = make_window()
    send(window, "go")
    assert wait_for(lambda: window.send_button.text() == "Stop" and len(window.chat.items) == 2)
    assert not window.new_chat_button.isEnabled()
    started = time.monotonic()
    QTest.mouseClick(window.send_button, Qt.MouseButton.LeftButton)
    assert idle(window)
    assert time.monotonic() - started < 3
    assert window.send_button.text() == "Send"
    assert window.chat.items[-1].text() == "Stopped."


def test_enter_sends_and_shift_enter_adds_a_line(make_window, llm_server):
    llm_server.script.append(text_reply("ok"))
    window = make_window()
    window.input.setFocus()
    QTest.keyClicks(window.input, "line one")
    QTest.keyClick(window.input, Qt.Key.Key_Return, Qt.KeyboardModifier.ShiftModifier)
    QTest.keyClicks(window.input, "line two")
    assert window.input.toPlainText() == "line one\nline two"
    assert not window.busy
    QTest.keyClick(window.input, Qt.Key.Key_Return)
    assert window.chat.items[0].text_so_far == "line one\nline two"
    assert idle(window)


def click_when_dialog_appears(button_text, seen):
    """Answers the modal confirmation once it opens (exec() blocks the test otherwise)."""
    def poll():
        dialog = QApplication.activeModalWidget()
        if dialog is None:
            QTimer.singleShot(20, poll)
            return
        seen.append(dialog)
        for button in dialog.findChildren(QPushButton):
            if button.text() == button_text:
                button.click()
    QTimer.singleShot(20, poll)


@pytest.mark.parametrize("answer, runs", [("Allow", True), ("Deny", False)])
def test_risky_command_asks_for_confirmation(make_window, api_server, llm_server, answer, runs):
    llm_server.script += [call_reply([("save_mesh", {"path": "/tmp/model.rmesh"})]), text_reply("ok")]
    window = make_window()
    seen = []
    click_when_dialog_appears(answer, seen)
    send(window, "save it")
    assert idle(window)
    assert seen and seen[0].objectName() == "confirm_dialog"
    assert "save_mesh" in seen[0].text()
    assert bool([c for c in api_server.calls if c[0] == "save_mesh"]) is runs
    assert window.chat.items[1].is_error is (not runs)


def test_confirmation_can_be_switched_off(make_window, api_server, llm_server):
    llm_server.script += [call_reply([("save_mesh", {"path": "x"})]), text_reply("ok")]
    window = make_window(confirm=lambda *args: pytest.fail("must not ask"))
    window.store.confirm_risky = False
    send(window, "save")
    assert idle(window)
    assert [c[0] for c in api_server.calls if c[0] == "save_mesh"] == ["save_mesh"]


def test_closing_with_a_request_in_flight_shuts_down_cleanly(make_window, llm_server):
    llm_server.script.append(text_reply("thinking", stall_after_first_chunk=True))
    window = make_window()
    send(window, "go")
    assert wait_for(lambda: len(window.chat.items) == 2)
    started = time.monotonic()
    window.close()
    assert time.monotonic() - started < 4
    assert not window._agent_thread.isRunning() and not window.poller.isRunning()


def test_new_chat_clears_everything(make_window, llm_server):
    llm_server.script += [call_reply([("screenshot", {})]), text_reply("ok"), text_reply("fresh")]
    window = make_window()
    send(window, "look")
    assert idle(window)
    window.new_chat_button.click()
    assert window.chat.items == [] and window.panel.images == []
    assert not window.undo_button.isEnabled()
    send(window, "again")
    assert idle(window)
    # the model saw no trace of the first conversation
    assert [m["role"] for m in llm_server.requests[-1]["body"]["messages"]] == ["system", "user"]


def test_save_conversation_writes_json_without_images(make_window, llm_server, tmp_path, monkeypatch):
    llm_server.script += [call_reply([("screenshot", {})]), text_reply("ok")]
    window = make_window(vision=True)
    send(window, "look")
    assert idle(window)
    target = tmp_path / "chat.json"
    monkeypatch.setattr(QFileDialog, "getSaveFileName", lambda *a, **k: (str(target), ""))
    window.save_button.click()
    data = json.loads(target.read_text())
    assert data["profile"] == "fake"
    assert [m["role"] for m in data["messages"]] == ["user", "assistant", "tool", "assistant"]
    assert data["messages"][2]["image"]["data"] == "<omitted>"


def test_profile_dialog_new_from_template_and_validation(app):
    dialog = ProfileDialog(None, "none", keyring_available=False)
    dialog.template.setCurrentText("DeepSeek")
    assert dialog.base_url.text() == "https://api.deepseek.com"
    assert dialog.api_key_env.text() == "DEEPSEEK_API_KEY"
    dialog.model.setText("")
    dialog._accept()
    assert "model" in dialog.error.text().lower() and dialog.result() == 0
    dialog.model.setText("deepseek-chat")
    dialog.api_key.setText("typed-secret")
    dialog.temperature_on.setChecked(True)
    dialog.temperature.setValue(0.3)
    dialog._accept()
    profile = dialog.profile()
    assert dialog.result() == 1
    assert (profile.name, profile.model, profile.temperature) == ("deepseek", "deepseek-chat", 0.3)
    assert dialog.typed_key() == "typed-secret"
    assert not dialog.remember.isEnabled()  # no keyring installed


def test_editing_a_profile_through_the_window_never_writes_the_key(make_window, tmp_path, monkeypatch):
    window = make_window()
    monkeypatch.setattr(ProfileDialog, "exec", lambda self: self.api_key.setText("sk-typed-123") or self.accept() or 1)
    window.edit_button.click()
    assert "sk-typed-123" not in (tmp_path / "profiles.json").read_text()
    assert window.secrets.resolve("llm:fake").value == "sk-typed-123"
