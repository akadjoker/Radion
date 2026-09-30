"""LLM profiles and app settings, stored as JSON in the user's config directory.

A Profile has no field for a key, so none can end up in the file: see secrets.py.
"""

import json
import os
import sys
from dataclasses import asdict, dataclass, fields
from pathlib import Path

from .api_client import DEFAULT_API_URL

KIND_OPENAI_COMPAT = "openai_compat"
FILE_VERSION = 1


def config_dir():
    """Per-user config directory (XDG on Linux, %APPDATA% on Windows, Application Support on macOS)."""
    if sys.platform == "win32":
        base = Path(os.environ.get("APPDATA") or Path.home() / "AppData" / "Roaming")
    elif sys.platform == "darwin":
        base = Path.home() / "Library" / "Application Support"
    else:
        base = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config")
    return base / "radion_chat"


@dataclass
class Profile:
    name: str
    kind: str = KIND_OPENAI_COMPAT
    base_url: str = ""
    model: str = ""
    api_key_env: str = ""          # name of the environment variable that holds the key
    vision: bool = False           # the user says the model accepts images
    simplify_schema: bool = False  # for servers that choke on oneOf/minItems/...
    stream: bool = True
    temperature: float | None = None  # None: the server's default
    max_steps: int = 40
    request_timeout: float = 300.0
    api_url: str = DEFAULT_API_URL
    context_chars: int = 120_000
    system_prompt_extra: str = ""

    def problems(self):
        """What must be fixed before this profile can be used (empty when fine)."""
        found = []
        if not self.name.strip():
            found.append("The profile needs a name.")
        if self.kind != KIND_OPENAI_COMPAT:
            found.append(f"Unknown provider kind '{self.kind}'.")
        if not self.base_url.startswith(("http://", "https://")):
            found.append("The base URL must start with http:// or https://.")
        if not self.model.strip():
            found.append("The model name is required.")
        if self.max_steps < 1:
            found.append("Max steps must be at least 1.")
        return found

    @classmethod
    def from_dict(cls, data):
        """Tolerant of a hand-edited file: unknown keys are ignored, wrong types fall back."""
        defaults = cls(name=str(data.get("name", "")))
        values = {}
        for field in fields(cls):
            if field.name in data and (data[field.name] is None or isinstance(
                    data[field.name], _accepted_types(getattr(defaults, field.name)))):
                values[field.name] = data[field.name]
        return cls(**{**asdict(defaults), **values})


def _accepted_types(default):
    if isinstance(default, bool):
        return bool
    if isinstance(default, (int, float)):
        return (int, float)
    if default is None:
        return (int, float)  # the only optional field is the temperature
    return type(default)


# Starting points offered by the "New profile" dialog; never saved on their own and never
# carrying a key. Model names change often: check the provider's current list.
TEMPLATES = {
    "Ollama (local)": Profile(
        name="ollama", base_url="http://localhost:11434/v1", model="", vision=False),
    "DeepSeek": Profile(
        name="deepseek", base_url="https://api.deepseek.com", model="deepseek-chat",
        api_key_env="DEEPSEEK_API_KEY", vision=False),
    "OpenAI": Profile(
        name="openai", base_url="https://api.openai.com/v1", model="",
        api_key_env="OPENAI_API_KEY", vision=True),
}


class ProfileStore:
    def __init__(self, path=None):
        self.path = Path(path) if path else config_dir() / "profiles.json"
        self.profiles = {}
        self.active = ""
        self.confirm_risky = True

    def load(self):
        """Reads the file; a missing or corrupt file just means an empty store."""
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return
        if not isinstance(data, dict):
            return
        self.profiles = {}
        for item in data.get("profiles", []):
            if isinstance(item, dict) and item.get("name"):
                profile = Profile.from_dict(item)
                self.profiles[profile.name] = profile
        self.active = data.get("active", "") if data.get("active") in self.profiles else ""
        self.confirm_risky = bool(data.get("confirm_risky", True))
        if not self.active and self.profiles:
            self.active = next(iter(self.profiles))

    def save(self):
        data = {
            "version": FILE_VERSION,
            "active": self.active,
            "confirm_risky": self.confirm_risky,
            "profiles": [asdict(p) for p in self.profiles.values()],
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(data, indent=2), encoding="utf-8")
        temporary.replace(self.path)  # atomic: a crash never leaves half a file

    def put(self, profile, replacing=None):
        """Adds or updates a profile (`replacing` is its old name when it was renamed)."""
        if replacing and replacing != profile.name:
            self.profiles.pop(replacing, None)
        self.profiles[profile.name] = profile
        if not self.active or self.active == replacing:
            self.active = profile.name

    def delete(self, name):
        self.profiles.pop(name, None)
        if self.active == name:
            self.active = next(iter(self.profiles), "")

    def current(self):
        return self.profiles.get(self.active)
