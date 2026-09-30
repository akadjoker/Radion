"""The agent loop: user message -> LLM -> tool calls -> editor API -> results -> LLM ...

It runs on whatever thread calls `run` (the UI uses a worker thread) and reports progress
through an AgentListener. Stop is a CancelToken, checked between steps and handed to the
provider so that it also interrupts a streaming reply.
"""

import json
from dataclasses import dataclass

from .api_client import ApiConnectionError, ApiError
from .history import prune_history, truncate
from .llm.base import Cancelled, CancelToken, LlmError
from .llm.openai_wire import assistant_to_history
from .prompts import build_system_prompt
from .tools import build_tools

# Commands that write files or discard work: the user confirms them by default.
RISKY_COMMANDS = frozenset({"save_mesh", "export_obj", "export_gltf", "new_document", "load_mesh"})

# `readOnly: false` commands that do NOT add an undo step in the editor (checked against the
# real editor: selection, visibility and the file writers leave the undo stack alone).
# Counting them would make "Undo this request" also undo the user's earlier work. A command
# listing with an explicit `undoable` field overrides this table.
NOT_UNDO_STEPS = frozenset({
    "select", "set_part_visible", "set_animation", "save_mesh", "export_obj", "export_gltf",
})

# These empty the editor's undo history, so nothing before them can be undone any more.
UNDO_BARRIERS = frozenset({"new_document", "load_mesh"})

UNDO_LIMIT_PER_CALL = 100  # the editor's `undo` accepts 1..100 steps

ERROR_HINTS = {
    "invalid_params": "Fix the arguments and try again.",
    "unknown_command": "That command does not exist; use one of the listed tools.",
    "failed": "The operation was not possible; the model is unchanged.",
    "timeout": "The editor did not answer in time (a dialog may be open in the editor). "
               "It is unknown whether the command ran: check with get_status before retrying.",
    "unavailable": "The editor is shutting down.",
}


@dataclass
class AgentConfig:
    max_steps: int = 40
    keep_images: int = 2          # only the latest screenshots stay attached to the history
    recent_results: int = 6       # newest tool results kept whole ...
    old_result_chars: int = 600   # ... older ones are cut to this many characters
    max_result_chars: int = 8000  # hard cap for any single result sent to the model
    max_history_chars: int = 120_000
    simplify_schema: bool = False  # see tools.simplify_schema
    extra_system_prompt: str = ""


class AgentListener:
    """Receives everything the agent does. Override what you need; all calls happen on the
    agent's thread."""

    def on_step(self, step, max_steps):
        pass

    def on_text_delta(self, text):
        pass

    def on_tool_call(self, call_id, name, arguments):
        pass

    def on_tool_result(self, call_id, name, summary, is_error, image_png, text):
        """`summary` is one short line; `text` is what the model received; `image_png` is
        bytes or None."""

    def on_error(self, message):
        pass


@dataclass
class RunResult:
    reason: str        # done | cancelled | max_steps | error
    steps: int = 0


@dataclass
class _Outcome:
    text: str
    is_error: bool = False
    summary: str = ""
    image_b64: str = ""
    image_png: bytes = b""
    fatal: bool = False  # the editor is unreachable: no point in carrying on


class Agent:
    def __init__(self, provider, api, listener=None, config=None, confirm=None):
        """`confirm(name, arguments) -> bool` is asked before RISKY_COMMANDS run (None: never)."""
        self.provider = provider
        self.api = api
        self.listener = listener or AgentListener()
        self.config = config or AgentConfig()
        self.confirm = confirm
        self.messages = []
        self.undoable_steps = 0  # editor undo steps the latest request added
        self._commands = {}

    def reset(self):
        """Starts a new conversation."""
        self.messages.clear()
        self.undoable_steps = 0

    # -- running a request -------------------------------------------------------------

    def run(self, user_text, cancel=None):
        cancel = cancel or CancelToken()
        try:
            commands = self.api.commands()
        except ApiError as error:
            self.listener.on_error(str(error))
            return RunResult("error")
        self._commands = {c["name"]: c for c in commands}
        tools = build_tools(commands, simplify=self.config.simplify_schema)

        self.undoable_steps = 0
        self.messages.append({"role": "user", "content": user_text})
        system = {"role": "system", "content": build_system_prompt(self.config.extra_system_prompt)}

        for step in range(1, self.config.max_steps + 1):
            if cancel.is_set:
                return RunResult("cancelled", step - 1)
            self.listener.on_step(step, self.config.max_steps)
            self._prune()
            try:
                reply = self.provider.complete(
                    [system, *self.messages], tools, self.listener.on_text_delta, cancel)
            except Cancelled:
                return RunResult("cancelled", step)
            except LlmError as error:
                self.listener.on_error(str(error))
                return RunResult("error", step)

            self.messages.append(assistant_to_history(reply))
            if not reply.tool_calls:
                if not reply.content.strip():
                    self.listener.on_error("The model returned an empty answer.")
                return RunResult("done", step)
            outcome = self._run_tool_calls(reply.tool_calls, cancel)
            if outcome:
                return RunResult(outcome, step)

        self.listener.on_error(
            f"Stopped after {self.config.max_steps} steps without a final answer. "
            "Send another message to let the model continue.")
        return RunResult("max_steps", self.config.max_steps)

    def _run_tool_calls(self, calls, cancel):
        """Executes one assistant turn's calls; returns 'cancelled'/'error' to end the run."""
        for position, call in enumerate(calls):
            if cancel.is_set:
                # Every tool_call needs an answer or the next request would be invalid.
                for skipped in calls[position:]:
                    self._add_tool_message(skipped.id, _Outcome("Cancelled by the user before it ran."))
                return "cancelled"
            outcome = self._execute(call)
            self._add_tool_message(call.id, outcome)
            if outcome.fatal:
                for skipped in calls[position + 1:]:
                    self._add_tool_message(skipped.id, _Outcome("Not run: the editor is unreachable."))
                return "error"
        return None

    def _execute(self, call):
        listener = self.listener
        listener.on_tool_call(call.id, call.name, call.arguments if call.arguments is not None else call.raw_arguments)
        outcome = self._attempt(call)
        listener.on_tool_result(call.id, call.name, outcome.summary or _summarize(outcome.text),
                                outcome.is_error, outcome.image_png or None, outcome.text)
        if outcome.fatal:
            listener.on_error(outcome.text)
        return outcome

    def _attempt(self, call):
        if call.error:
            return _Outcome(f"ERROR: {call.error}. Send the arguments as one JSON object.", True)
        if call.name not in self._commands:
            names = ", ".join(sorted(self._commands))
            return _Outcome(f"ERROR: unknown command '{call.name}'. Available commands: {names}.", True)
        if call.name in RISKY_COMMANDS and self.confirm and not self.confirm(call.name, call.arguments):
            return _Outcome(f"The user declined to run {call.name}. Do not retry it; "
                            "continue without it or ask the user.", True)
        try:
            reply = self.api.call(call.name, call.arguments)
        except ApiConnectionError as error:
            return _Outcome(f"ERROR: {error}", True, fatal=True)
        except ApiError as error:
            return _Outcome(_error_text(error), True)

        self._count_undo_steps(call.name, call.arguments, reply.result)
        text = truncate(json.dumps(reply.result, separators=(",", ":")), self.config.max_result_chars)
        outcome = _Outcome(text, summary=_summarize(text))
        if reply.image_b64:
            outcome.image_png = reply.image_bytes
            if self.provider.supports_images:
                outcome.image_b64 = reply.image_b64
            else:
                outcome.text += ("\n(A screenshot was taken but you cannot see images: "
                                 "rely on get_status instead.)")
        return outcome

    def _add_tool_message(self, call_id, outcome):
        message = {"role": "tool", "tool_call_id": call_id, "content": outcome.text}
        if outcome.image_b64:
            message["image"] = {"mimeType": "image/png", "data": outcome.image_b64}
        self.messages.append(message)

    def _prune(self):
        config = self.config
        prune_history(self.messages, keep_images=config.keep_images,
                      recent_results=config.recent_results,
                      old_result_chars=config.old_result_chars,
                      max_chars=config.max_history_chars)

    # -- undo --------------------------------------------------------------------------

    def _count_undo_steps(self, name, arguments, result):
        if name in UNDO_BARRIERS:
            self.undoable_steps = 0
        elif name == "undo":
            self.undoable_steps = max(0, self.undoable_steps - result.get("stepsUndone", 0))
        elif name == "redo":
            self.undoable_steps += result.get("stepsRedone", 0)
        else:
            command = self._commands[name]
            undoable = command.get("undoable", name not in NOT_UNDO_STEPS)
            if not command.get("readOnly", False) and undoable:
                self.undoable_steps += 1

    def undo_last_request(self):
        """Reverts the edits the latest request made; returns how many steps were undone."""
        remaining, undone = self.undoable_steps, 0
        while remaining > 0:
            chunk = min(remaining, UNDO_LIMIT_PER_CALL)
            reply = self.api.call("undo", {"steps": chunk})
            done = reply.result.get("stepsUndone", chunk)
            undone += done
            remaining -= chunk
            if done < chunk:
                break
        self.undoable_steps = 0
        if undone:
            self.messages.append({"role": "user", "content": (
                f"[Client note] The user pressed Undo: the {undone} edit(s) of your previous "
                "request were reverted. The model is back to how it was before that request.")})
        return undone

    # -- saving ------------------------------------------------------------------------

    def export_conversation(self):
        """The conversation as plain JSON-able data (images replaced by a placeholder)."""
        messages = []
        for message in self.messages:
            message = dict(message)
            if "image" in message:
                message["image"] = {"mimeType": message["image"]["mimeType"], "data": "<omitted>"}
            messages.append(message)
        return {"system_prompt": build_system_prompt(self.config.extra_system_prompt),
                "messages": messages}


def _error_text(error):
    hint = ERROR_HINTS.get(error.code, "")
    if error.http_status == 401:
        hint = "The editor requires an API token that the client does not have or that is wrong."
    return f"ERROR {error.code} (HTTP {error.http_status}): {error.message}. {hint}".strip()


def _summarize(text):
    """One line for the UI: the result JSON, shortened."""
    return text if len(text) <= 160 else text[:157] + "..."
