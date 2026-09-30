"""Runs the agent and the health polling off the GUI thread.

Both objects are moved to their own QThread by the main window; everything the GUI needs
to know comes back as signals, and the only calls made *directly* from the GUI thread are
the thread-safe ones (`submit`, `cancel`, `answer_confirm`).
"""

import threading

from PySide6.QtCore import QObject, QThread, Signal, Slot

from ..agent import AgentListener
from ..api_client import ApiError, RadionApiClient
from ..llm.base import CancelToken, LlmError
from ..session import make_agent


class AgentWorker(QObject, AgentListener):
    # Signals carrying what the agent reports (the AgentListener methods emit them).
    step = Signal(int, int)
    text_delta = Signal(str)
    tool_call = Signal(str, str, object)                     # id, name, arguments
    tool_result = Signal(str, str, str, bool, object, str)   # id, name, summary, is_error, png bytes|None, text
    error = Signal(str)
    finished = Signal(str, int)                              # reason, undoable steps
    undone = Signal(int)                                     # steps undone by "Undo this request"
    confirm_requested = Signal(str, object)                  # command, arguments

    # Requests from the GUI thread, executed on the worker thread (queued connections).
    _configure_requested = Signal(object)
    _run_requested = Signal(str)
    _undo_requested = Signal()
    _reset_requested = Signal()

    def __init__(self):
        super().__init__()
        self._agent = None
        self._cancel = CancelToken()
        self._confirm_event = threading.Event()
        self._confirm_answer = False
        self._configure_requested.connect(self._configure)
        self._run_requested.connect(self._run)
        self._undo_requested.connect(self._undo)
        self._reset_requested.connect(self._reset)

    # -- called from the GUI thread ----------------------------------------------------

    def configure(self, profile, api_key, api_token, confirm_risky):
        self._configure_requested.emit((profile, api_key, api_token, confirm_risky))

    def submit(self, text):
        self._cancel = CancelToken()  # made here, so a Stop before the run starts still counts
        self._run_requested.emit(text)

    def cancel(self):
        self._cancel.cancel()
        self._confirm_answer = False
        self._confirm_event.set()

    def answer_confirm(self, allowed):
        self._confirm_answer = allowed
        self._confirm_event.set()

    def request_undo(self):
        self._undo_requested.emit()

    def request_reset(self):
        self._reset_requested.emit()

    def conversation(self):
        """The conversation to save; only call while no request is running."""
        return self._agent.export_conversation() if self._agent else {"system_prompt": "", "messages": []}

    # -- AgentListener: forwarded as signals ------------------------------------------

    def on_step(self, step, max_steps):
        self.step.emit(step, max_steps)

    def on_text_delta(self, text):
        self.text_delta.emit(text)

    def on_tool_call(self, call_id, name, arguments):
        self.tool_call.emit(call_id, name, arguments)

    def on_tool_result(self, call_id, name, summary, is_error, image_png, text):
        self.tool_result.emit(call_id, name, summary, is_error, image_png, text)

    def on_error(self, message):
        self.error.emit(message)

    # -- on the worker thread ----------------------------------------------------------

    @Slot(object)
    def _configure(self, settings):
        profile, api_key, api_token, confirm_risky = settings
        try:
            agent = make_agent(profile, api_key, api_token, self,
                               self._ask_user if confirm_risky else None)
        except LlmError as problem:
            self.error.emit(str(problem))
            self._agent = None
            return
        if self._agent:  # switching profile keeps the conversation
            agent.messages = self._agent.messages
            agent.undoable_steps = self._agent.undoable_steps
        self._agent = agent

    @Slot(str)
    def _run(self, text):
        if not self._agent:
            self.finished.emit("error", 0)
            return
        try:
            result = self._agent.run(text, self._cancel)
        except Exception as bug:  # a thread must never die silently: tell the user
            self.error.emit(f"Internal error: {bug!r}")
            self.finished.emit("error", self._agent.undoable_steps)
            return
        self.finished.emit(result.reason, self._agent.undoable_steps)

    @Slot()
    def _undo(self):
        if not self._agent:
            return
        try:
            self.undone.emit(self._agent.undo_last_request())
        except ApiError as problem:
            self.error.emit(f"Undo failed: {problem}")
            self.undone.emit(0)

    @Slot()
    def _reset(self):
        if self._agent:
            self._agent.reset()

    def _ask_user(self, name, arguments):
        """Blocks the agent thread until the GUI answers (or Stop is pressed)."""
        self._confirm_event.clear()
        self.confirm_requested.emit(name, arguments)
        while not self._confirm_event.wait(0.1):
            if self._cancel.is_set:
                return False
        return self._confirm_answer


class HealthPoller(QThread):
    """Polls `/api/health` so the status bar shows whether the editor is reachable."""

    state = Signal(bool, str)   # reachable, text for the status bar

    def __init__(self, interval_s=2.0):
        super().__init__()
        self._interval_s = interval_s
        self._client = RadionApiClient()
        self._wake = threading.Event()
        self._stopping = False

    def set_target(self, url, token):
        """Points the poller at another editor and polls right away (any thread)."""
        self._client = RadionApiClient(url, token)
        self._wake.set()

    def stop(self):
        self._stopping = True
        self._wake.set()

    def run(self):
        while not self._stopping:
            self._wake.clear()
            self._poll()
            self._wake.wait(self._interval_s)

    def _poll(self):
        client = self._client
        try:
            client.health(timeout=1.5)
        except ApiError as problem:
            self.state.emit(False, str(problem))
        else:
            self.state.emit(True, f"Editor API connected ({client.base_url})")
