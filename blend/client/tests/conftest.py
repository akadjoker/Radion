import json
import sys
from pathlib import Path

import pytest

TESTS = Path(__file__).parent
sys.path.insert(0, str(TESTS))  # the fake servers live next to the tests

from fake_llm_server import FakeLlmServer  # noqa: E402
from fake_radion_api import FakeRadionApi  # noqa: E402


@pytest.fixture
def commands():
    return json.loads((TESTS / "fixtures" / "commands.json").read_text())["commands"]


@pytest.fixture
def api_server():
    server = FakeRadionApi().start()
    yield server
    server.stop()


@pytest.fixture
def llm_server():
    server = FakeLlmServer().start()
    yield server
    server.stop()
