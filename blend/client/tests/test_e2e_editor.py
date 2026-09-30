"""End-to-end: the agent (scripted fake LLM) against the real radion_blender editor.

Skipped unless an editor is available:
  RADION_E2E_API_URL=http://127.0.0.1:7431   use an editor that is already running, or
  RADION_EDITOR_BIN=/path/to/radion_blender  start one headless (needs xvfb-run).
No Qt is involved: this is the agent loop over the real HTTP API.
"""

import os
import shutil
import signal
import socket
import struct
import subprocess
import time

import pytest
from fake_llm_server import FakeLlmServer, call_reply, text_reply
from radion_chat.agent import NOT_UNDO_STEPS, AgentListener
from radion_chat.api_client import ApiError, RadionApiClient
from radion_chat.profiles import Profile
from radion_chat.session import make_agent

pytestmark = pytest.mark.e2e


def _free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def _wait_for_health(url, timeout=90):
    client = RadionApiClient(url)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            client.health()
            return True
        except ApiError:
            time.sleep(1)
    return False


@pytest.fixture(scope="module")
def editor_url():
    existing = os.environ.get("RADION_E2E_API_URL")
    if existing:
        yield existing
        return
    binary = os.environ.get("RADION_EDITOR_BIN")
    if not binary or not shutil.which("xvfb-run"):
        pytest.skip("set RADION_E2E_API_URL or RADION_EDITOR_BIN (and install xvfb-run) to run the e2e tests")
    port = _free_port()
    environment = {**os.environ, "MESA_GL_VERSION_OVERRIDE": "4.5", "LIBGL_ALWAYS_SOFTWARE": "1"}
    process = subprocess.Popen(
        ["xvfb-run", "-a", "-s", "-screen 0 1920x1080x24", binary, "--api-port", str(port)],
        env=environment, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)  # own process group: xvfb-run, Xvfb and the editor die together
    url = f"http://127.0.0.1:{port}"
    try:
        if not _wait_for_health(url):
            pytest.fail("the editor did not start answering /api/health")
        yield url
    finally:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=30)


@pytest.fixture
def editor(editor_url):
    client = RadionApiClient(editor_url)
    client.call("new_document")
    return client


class Capture(AgentListener):
    def __init__(self):
        self.images = []
        self.results = []

    def on_tool_result(self, call_id, name, summary, is_error, image_png, text):
        self.results.append((name, is_error, text))
        if image_png:
            self.images.append(image_png)


def part_names(client):
    return [part["name"] for part in client.call("get_status").result.get("parts", [])]


def png_size(data):
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    return struct.unpack(">II", data[16:24])


@pytest.fixture
def scripted_agent(editor_url):
    llm = FakeLlmServer().start()
    profile = Profile(name="e2e", base_url=llm.base_url, model="fake", api_url=editor_url, vision=True)
    capture = Capture()
    agent = make_agent(profile, listener=capture)
    yield agent, llm, capture
    llm.stop()


def test_agent_builds_checks_and_undoes_against_the_real_editor(editor, scripted_agent):
    agent, llm, capture = scripted_agent
    editor.call("add_primitive", {"type": "sphere", "name": "earlier_work"})  # the user's own edit

    sections = [{"at": -1.0, "width": 0.2, "height": 0.2}, {"at": 0.0, "width": 1.0, "height": 0.8},
                {"at": 1.0, "width": 0.4, "height": 0.4}]
    llm.script += [
        call_reply([("add_primitive", {"type": "box", "name": "hull", "color": "#556b2f", "scale": [2, 0.6, 3.5]}),
                    ("add_loft", {"name": "nose", "axis": "z", "position": [0, 0.3, 2], "sections": sections})],
                   text="Building the hull and a nose."),
        call_reply([("select", {"action": "all"}), ("get_status", {}),
                    ("screenshot", {"width": 320, "height": 240, "view": "right"})]),
        call_reply([("add_primitive", {"type": "teapot"})]),  # invalid: the editor refuses it
        text_reply("Built a hull with a nose."),
    ]
    result = agent.run("make a tank")

    assert (result.reason, result.steps) == ("done", 4)
    assert part_names(editor) == ["earlier_work", "hull", "nose"]
    live = editor.call("get_status").result
    assert [p["triangles"] for p in live["parts"]][1] > 0 and live["parts"][2]["triangles"] > 0

    assert len(capture.images) == 1
    assert png_size(capture.images[0]) == (320, 240)
    failed = [r for r in capture.results if r[1]]
    assert len(failed) == 1 and failed[0][0] == "add_primitive"
    assert "must be one of" in failed[0][2]

    # The model got every real command as a tool, and the real error text back.
    assert len(llm.requests[0]["body"]["tools"]) == len(editor.commands())
    last_messages = llm.requests[-1]["body"]["messages"]
    assert any(m["role"] == "tool" and "invalid_params" in m["content"] for m in last_messages)
    screenshot_follow_up = [m for m in last_messages if isinstance(m["content"], list)]
    assert screenshot_follow_up and screenshot_follow_up[0]["content"][1]["image_url"]["url"].startswith(
        "data:image/png;base64,")

    # Two parts were added; select, get_status, screenshot and the failed call count for nothing.
    assert agent.undoable_steps == 2
    assert agent.undo_last_request() == 2
    assert part_names(editor) == ["earlier_work"]
    assert editor.call("get_status").result["canUndo"] is True  # the user's own edit is intact


def test_commands_that_are_not_undo_steps_really_are_not(editor):
    """agent.NOT_UNDO_STEPS is knowledge about the editor: keep it honest."""
    scratch = os.environ.get("TMPDIR", "/tmp")
    arguments = {
        "select": {"action": "all"},
        "set_part_visible": {"part": 0, "visible": True},
        "save_mesh": {"path": f"{scratch}/radion_chat_e2e.rmesh"},
        "export_obj": {"path": f"{scratch}/radion_chat_e2e.obj"},
        "export_gltf": {"path": f"{scratch}/radion_chat_e2e.glb"},
    }
    checked = 0
    for name, args in arguments.items():
        assert name in NOT_UNDO_STEPS
        editor.call("new_document")
        editor.call("add_primitive", {"type": "box", "name": "a"})
        editor.call(name, args)
        editor.call("undo", {"steps": 1})
        # One undo removed the box, so `name` had pushed no undo step of its own.
        assert editor.call("get_status").result["hasMesh"] is False, name
        checked += 1
    assert checked == len(arguments)


def test_undo_is_clamped_to_what_exists(editor):
    editor.call("add_primitive", {"type": "box", "name": "a"})
    assert editor.call("undo", {"steps": 50}).result["stepsUndone"] == 1
