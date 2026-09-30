"""The interface every LLM backend implements, and the types that cross it.

Conversation messages use the OpenAI chat shape (role/content/tool_calls/tool_call_id)
because every provider planned for this client can be mapped from it. A `tool` message may
also carry an `image` ({"mimeType", "data"(base64)}) key; the provider decides how to show
it to the model (or drops it when the model cannot see images).
"""

import threading
from abc import ABC, abstractmethod
from contextlib import contextmanager
from dataclasses import dataclass, field


class LlmError(Exception):
    """The model backend failed (network, HTTP error, unusable answer)."""

    def __init__(self, message, http_status=None):
        super().__init__(message)
        self.http_status = http_status


class Cancelled(Exception):
    """The user pressed Stop."""


class CancelToken:
    """Thread-safe Stop flag that can also interrupt a blocking network read.

    A provider registers a callback with `on_cancel` for the duration of a request; Stop
    then shuts the socket down from the GUI thread, which wakes the reading thread at once
    instead of after the next SSE line (a model may think for minutes between lines).
    """

    def __init__(self):
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._callbacks = []

    @property
    def is_set(self):
        return self._event.is_set()

    def cancel(self):
        with self._lock:
            self._event.set()
            callbacks = list(self._callbacks)
        for callback in callbacks:
            callback()

    def wait(self, timeout):
        """Sleeps up to `timeout` seconds; True as soon as Stop is pressed."""
        return self._event.wait(timeout)

    def raise_if_set(self):
        if self._event.is_set():
            raise Cancelled()

    @contextmanager
    def on_cancel(self, callback):
        with self._lock:
            already = self._event.is_set()
            if not already:
                self._callbacks.append(callback)
        if already:
            callback()
        try:
            yield
        finally:
            with self._lock:
                if callback in self._callbacks:
                    self._callbacks.remove(callback)


@dataclass
class ToolCall:
    """One function call requested by the model.

    `arguments` is None when the model sent something that is not a JSON object; `error`
    then says why, and the agent reports it back to the model instead of executing it.
    """

    id: str
    name: str
    arguments: dict | None
    raw_arguments: str = ""
    error: str = ""


@dataclass
class Usage:
    prompt_tokens: int = 0
    completion_tokens: int = 0


@dataclass
class AssistantMessage:
    content: str = ""
    tool_calls: list = field(default_factory=list)
    finish_reason: str = ""
    usage: Usage | None = None


class LlmProvider(ABC):
    @property
    @abstractmethod
    def supports_images(self):
        """True when screenshots may be sent to the model."""

    @abstractmethod
    def complete(self, messages, tools, on_text_delta=None, cancel=None):
        """Sends the conversation and returns the assistant's reply.

        `on_text_delta(str)` is called for text as it streams in (once with everything for
        providers that do not stream). `cancel` is a CancelToken; raise Cancelled when it
        fires. Raise LlmError for any other failure.
        """
