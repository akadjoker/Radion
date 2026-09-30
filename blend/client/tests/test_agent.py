import json
import threading

import pytest
from fake_llm_server import call_reply, text_reply
from radion_chat.agent import Agent, AgentConfig, AgentListener
from radion_chat.api_client import RadionApiClient
from radion_chat.llm.base import CancelToken
from radion_chat.llm.openai_compat import OpenAICompatProvider


class Recorder(AgentListener):
    def __init__(self):
        self.events = []
        self.on_result = None

    def on_step(self, step, max_steps):
        self.events.append(("step", step))

    def on_text_delta(self, text):
        self.events.append(("text", text))

    def on_tool_call(self, call_id, name, arguments):
        self.events.append(("call", name, arguments))

    def on_tool_result(self, call_id, name, summary, is_error, image_png, text):
        self.events.append(("result", name, is_error, bool(image_png)))
        if self.on_result:
            self.on_result(name)

    def on_error(self, message):
        self.events.append(("error", message))

    def kinds(self, kind):
        return [e for e in self.events if e[0] == kind]


@pytest.fixture
def make_agent(api_server, llm_server):
    def factory(vision=False, confirm=None, **config):
        listener = Recorder()
        provider = OpenAICompatProvider(llm_server.base_url, "m", vision=vision)
        agent = Agent(provider, RadionApiClient(api_server.url), listener, AgentConfig(**config), confirm)
        return agent, listener
    return factory


def add_box(name="box"):
    return ("add_primitive", {"type": "box", "name": name})


def assert_history_is_valid(messages):
    pending = set()
    for message in messages:
        if message["role"] == "assistant":
            assert not pending
            pending = {c["id"] for c in message.get("tool_calls", [])}
        elif message["role"] == "tool":
            assert message["tool_call_id"] in pending
            pending.discard(message["tool_call_id"])
        else:
            assert not pending
    assert not pending


def test_full_loop_builds_checks_and_answers(make_agent, api_server, llm_server):
    llm_server.script += [
        call_reply([add_box("hull"), ("add_loft", {"name": "nose", "sections": [{"at": 0}]})], text="Building."),
        call_reply([("get_status", {}), ("screenshot", {"view": "right"})]),
        text_reply("A hull and a nose."),
    ]
    agent, listener = make_agent()
    result = agent.run("make a tank")

    assert (result.reason, result.steps) == ("done", 3)
    assert api_server.names() == ["hull", "nose"]
    assert [c[1] for c in listener.kinds("call")] == ["add_primitive", "add_loft", "get_status", "screenshot"]
    assert listener.kinds("call")[0][2] == {"type": "box", "name": "hull"}
    assert [e[2] for e in listener.kinds("result")] == [False] * 4
    assert listener.kinds("result")[3][3] is True
    assert "".join(e[1] for e in listener.kinds("text")).endswith("A hull and a nose.")
    assert [e[1] for e in listener.kinds("step")] == [1, 2, 3]
    assert_history_is_valid(agent.messages)
    assert agent.messages[0] == {"role": "user", "content": "make a tank"}


def test_each_request_carries_system_prompt_tools_and_results(make_agent, llm_server):
    llm_server.script += [call_reply([("get_status", {})]), text_reply("ok")]
    agent, _ = make_agent()
    agent.run("hello")
    first, second = (r["body"] for r in llm_server.requests)
    assert first["messages"][0]["role"] == "system"
    assert "+Y is up" in first["messages"][0]["content"]
    assert len(first["tools"]) == 39
    roles = [m["role"] for m in second["messages"]]
    assert roles == ["system", "user", "assistant", "tool"]
    assert json.loads(second["messages"][3]["content"])["hasMesh"] is False
    assert second["messages"][2]["tool_calls"][0]["function"]["arguments"] == "{}"


def test_step_limit(make_agent, llm_server):
    llm_server.script += [call_reply([("get_status", {})]) for _ in range(10)]
    agent, listener = make_agent(max_steps=3)
    result = agent.run("loop forever")
    assert (result.reason, result.steps) == ("max_steps", 3)
    assert len(llm_server.requests) == 3
    assert "3 steps" in listener.kinds("error")[0][1]
    assert_history_is_valid(agent.messages)


def test_empty_answer_is_reported(make_agent, llm_server):
    llm_server.script.append(text_reply(""))
    agent, listener = make_agent()
    assert agent.run("hi").reason == "done"
    assert "empty" in listener.kinds("error")[0][1]


def test_llm_failure_ends_the_run_with_an_error(make_agent, llm_server):
    agent, listener = make_agent()  # empty script: the fake answers HTTP 500
    assert agent.run("hi").reason == "error"
    assert "500" in listener.kinds("error")[0][1]


def test_editor_not_running(make_agent, api_server, llm_server):
    api_server.stop()
    agent, listener = make_agent()
    assert agent.run("hi").reason == "error"
    assert "radion_blender --api" in listener.kinds("error")[0][1]
    assert llm_server.requests == []


def test_editor_dying_mid_run_ends_it_and_keeps_history_valid(make_agent, api_server, llm_server):
    llm_server.script += [call_reply([("get_status", {}), ("get_status", {})])]
    agent, listener = make_agent()
    listener.on_result = lambda name: api_server.stop()
    assert agent.run("hi").reason == "error"
    assert len(llm_server.requests) == 1
    assert_history_is_valid(agent.messages)


def test_api_error_is_told_to_the_model_which_corrects_itself(make_agent, api_server, llm_server):
    llm_server.script += [
        call_reply([("add_primitive", {"type": "teapot", "name": "pot"})]),
        call_reply([add_box("pot")]),
        text_reply("Fixed."),
    ]
    agent, listener = make_agent()
    assert agent.run("a pot").reason == "done"

    second = llm_server.requests[1]["body"]["messages"]
    error_text = second[-1]["content"]
    assert error_text.startswith("ERROR invalid_params (HTTP 400)")
    assert "must be one of" in error_text and "Fix the arguments" in error_text
    assert [e[2] for e in listener.kinds("result")] == [True, False]
    assert api_server.names() == ["pot"]


def test_malformed_arguments_are_reported_without_calling_the_api(make_agent, api_server, llm_server):
    llm_server.script += [call_reply([("add_primitive", '{"type": "box",')]), text_reply("sorry")]
    agent, listener = make_agent()
    assert agent.run("box").reason == "done"
    assert api_server.calls == []
    tool_message = llm_server.requests[1]["body"]["messages"][-1]
    assert "not valid JSON" in tool_message["content"]
    assert listener.kinds("result")[0][2] is True
    # The broken text is not replayed to the server: it would reject the history.
    assert llm_server.requests[1]["body"]["messages"][-2]["tool_calls"][0]["function"]["arguments"] == "{}"


def test_unknown_command_lists_the_real_ones(make_agent, api_server, llm_server):
    llm_server.script += [call_reply([("make_tank", {})]), text_reply("ok")]
    agent, _ = make_agent()
    agent.run("tank")
    content = llm_server.requests[1]["body"]["messages"][-1]["content"]
    assert "unknown command 'make_tank'" in content and "add_primitive" in content
    assert not [c for c in api_server.calls if c[0] == "make_tank"]


def test_huge_results_are_capped(make_agent, llm_server):
    llm_server.script += [call_reply([("get_mesh_data", {})]), text_reply("ok")]
    agent, _ = make_agent(max_result_chars=500)
    agent.run("dump")
    content = llm_server.requests[1]["body"]["messages"][-1]["content"]
    assert len(content) < 600 and "truncated" in content


def test_declined_risky_command_is_not_run(make_agent, api_server, llm_server):
    llm_server.script += [call_reply([("save_mesh", {"path": "/tmp/x.rmesh"})]), text_reply("ok")]
    asked = []
    agent, listener = make_agent(confirm=lambda name, args: asked.append((name, args)) or False)
    agent.run("save it")
    assert asked == [("save_mesh", {"path": "/tmp/x.rmesh"})]
    assert not [c for c in api_server.calls if c[0] == "save_mesh"]
    assert "declined" in llm_server.requests[1]["body"]["messages"][-1]["content"]
    assert listener.kinds("result")[0][2] is True


def test_approved_risky_command_runs_and_safe_ones_are_not_asked(make_agent, api_server, llm_server):
    llm_server.script += [call_reply([("save_mesh", {"path": "/tmp/x.rmesh"}), ("get_status", {})]), text_reply("ok")]
    asked = []
    agent, _ = make_agent(confirm=lambda name, args: asked.append(name) or True)
    agent.run("save it")
    assert asked == ["save_mesh"]
    assert [c[0] for c in api_server.calls if c[0] in ("save_mesh", "get_status")] == ["save_mesh", "get_status"]


def test_cancel_between_steps(make_agent, llm_server):
    llm_server.script += [call_reply([("get_status", {})]), text_reply("never asked")]
    agent, listener = make_agent()
    cancel = CancelToken()
    listener.on_result = lambda name: cancel.cancel()
    result = agent.run("go", cancel)
    assert result.reason == "cancelled"
    assert len(llm_server.requests) == 1
    assert_history_is_valid(agent.messages)


def test_cancel_inside_a_batch_skips_the_remaining_calls(make_agent, api_server, llm_server):
    llm_server.script += [call_reply([add_box("a"), add_box("b"), add_box("c")])]
    agent, listener = make_agent()
    cancel = CancelToken()
    listener.on_result = lambda name: cancel.cancel()
    assert agent.run("three boxes", cancel).reason == "cancelled"
    assert api_server.names() == ["a"]
    assert agent.undoable_steps == 1
    assert_history_is_valid(agent.messages)
    assert "Cancelled" in agent.messages[-1]["content"]


def test_cancel_while_the_model_is_streaming(make_agent, llm_server):
    llm_server.script.append(text_reply("thinking", stall_after_first_chunk=True))
    agent, listener = make_agent()
    cancel = CancelToken()
    outcome = []
    thread = threading.Thread(target=lambda: outcome.append(agent.run("go", cancel)))
    thread.start()
    for _ in range(100):
        if listener.kinds("text"):
            break
        threading.Event().wait(0.05)
    cancel.cancel()
    thread.join(5)
    assert not thread.is_alive()
    assert outcome[0].reason == "cancelled"


def test_the_agent_can_be_used_again_after_a_cancel(make_agent, llm_server):
    llm_server.script += [call_reply([("get_status", {})]), text_reply("fine")]
    agent, listener = make_agent()
    cancel = CancelToken()
    listener.on_result = lambda name: cancel.cancel()
    agent.run("first", cancel)
    listener.on_result = None
    assert agent.run("second").reason == "done"
    assert_history_is_valid(agent.messages)


def test_old_big_results_are_truncated_and_only_latest_screenshots_stay(make_agent, llm_server):
    llm_server.script += [call_reply([("get_mesh_data", {})]) for _ in range(3)]
    llm_server.script += [call_reply([("screenshot", {})]) for _ in range(4)]
    llm_server.script += [text_reply("done")]
    agent, _ = make_agent(vision=True, recent_results=2, old_result_chars=100, keep_images=2)
    agent.run("inspect")

    tools = [m for m in agent.messages if m["role"] == "tool"]
    assert len(tools) == 7
    assert all(len(m["content"]) < 200 for m in tools[:3])
    assert "omitted from history" in tools[0]["content"]
    assert [bool(m.get("image")) for m in tools[3:]] == [False, False, True, True]
    last_request = llm_server.requests[-1]["body"]["messages"]
    images = [m for m in last_request if isinstance(m["content"], list)]
    assert len(images) == 2
    assert_history_is_valid(agent.messages)


def test_history_is_bounded_by_dropping_oldest_turns(make_agent, llm_server):
    llm_server.script += [text_reply("x" * 400) for _ in range(6)]
    agent, _ = make_agent(max_history_chars=1500)
    for index in range(6):
        agent.run(f"request {index} " + "y" * 200)
    assert agent.messages[0]["role"] == "user"
    assert agent.messages[-2]["content"].startswith("request 5")
    assert len(agent.messages) < 12
    sent = llm_server.requests[-1]["body"]["messages"]
    assert sum(len(m["content"]) for m in sent[1:]) < 2200


def test_vision_off_keeps_images_out_of_the_model_context(make_agent, llm_server):
    llm_server.script += [call_reply([("screenshot", {})]), text_reply("ok")]
    agent, listener = make_agent(vision=False)
    agent.run("look")
    tool_message = next(m for m in agent.messages if m["role"] == "tool")
    assert "image" not in tool_message
    assert "cannot see images" in tool_message["content"]
    assert listener.kinds("result")[0][3] is True
    assert not any(isinstance(m["content"], list) for m in llm_server.requests[1]["body"]["messages"])


def test_exported_conversation_has_no_image_data(make_agent, llm_server):
    llm_server.script += [call_reply([("screenshot", {})]), text_reply("ok")]
    agent, _ = make_agent(vision=True)
    agent.run("look")
    exported = json.dumps(agent.export_conversation())
    assert "<omitted>" in exported
    assert agent.messages[2]["image"]["data"] not in exported
    assert json.loads(exported)["messages"][0]["content"] == "look"


def test_undo_request_counts_only_edits_that_succeeded_and_made_undo_steps(make_agent, api_server, llm_server):
    llm_server.script += [
        call_reply([add_box("a"), add_box("b"), ("select", {"action": "all"}),
                    ("set_part_visible", {"part": 0, "visible": True}),
                    ("add_primitive", {"type": "teapot"}),          # fails: changed nothing
                    ("get_status", {}), ("save_mesh", {"path": "x"})]),
        text_reply("done"),
    ]
    agent, _ = make_agent()
    agent.run("two boxes")
    assert api_server.names() == ["a", "b"]
    assert agent.undoable_steps == 2

    assert agent.undo_last_request() == 2
    assert api_server.names() == []
    assert agent.undoable_steps == 0
    assert agent.undo_last_request() == 0
    assert "Undo" in agent.messages[-1]["content"] and agent.messages[-1]["role"] == "user"


def test_undo_request_leaves_earlier_work_alone(make_agent, api_server, llm_server):
    llm_server.script += [call_reply([add_box("first")]), text_reply("ok"),
                          call_reply([add_box("second"), add_box("third")]), text_reply("ok")]
    agent, _ = make_agent()
    agent.run("one")
    agent.run("two")
    assert agent.undoable_steps == 2
    agent.undo_last_request()
    assert api_server.names() == ["first"]


def test_undo_and_redo_by_the_model_adjust_the_count(make_agent, api_server, llm_server):
    llm_server.script += [
        call_reply([add_box("a"), add_box("b"), add_box("c"), ("undo", {"steps": 2}), ("redo", {"steps": 1})]),
        text_reply("ok")]
    agent, _ = make_agent()
    agent.run("x")
    assert api_server.names() == ["a", "b"]
    assert agent.undoable_steps == 2


def test_new_document_and_load_mesh_are_undo_barriers(make_agent, api_server, llm_server):
    llm_server.script += [call_reply([add_box("a"), ("new_document", {}), add_box("b")]), text_reply("ok")]
    agent, _ = make_agent()
    agent.run("x")
    assert agent.undoable_steps == 1  # only what came after the barrier can be undone
    agent.undo_last_request()
    assert api_server.names() == []


def test_undo_uses_chunks_of_at_most_100_steps(make_agent, api_server, commands):
    agent, _ = make_agent()
    agent._commands = {c["name"]: c for c in commands}
    for _ in range(250):
        api_server.cmd_add_primitive({"type": "box"})  # edits made behind the client's back ...
        agent._count_undo_steps("add_primitive", {}, {})  # ... and counted by it
    assert agent.undo_last_request() == 250
    assert [args["steps"] for name, args in api_server.calls if name == "undo"] == [100, 100, 50]
    assert api_server.names() == []
