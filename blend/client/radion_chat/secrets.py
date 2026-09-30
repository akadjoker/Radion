"""Where API keys come from. They are never written to the profiles file.

Order of resolution: environment variable named by the profile, then the system keyring
(only if the optional `keyring` package is installed and has the key), then a value typed
in the UI, which lives in memory only and is gone when the app closes.
"""

import os
from dataclasses import dataclass

KEYRING_SERVICE = "radion_chat"
API_TOKEN_ACCOUNT = "radion-editor-api-token"
API_TOKEN_ENV = "RADION_BLENDER_API_TOKEN"


def llm_key_account(profile_name):
    return f"llm:{profile_name}"


@dataclass
class ResolvedSecret:
    value: str
    source: str  # "env" | "keyring" | "memory" | "none"


def _load_keyring():
    """The keyring module, or None when it is not installed (it is optional)."""
    try:
        import keyring
    except ImportError:
        return None
    return keyring


class SecretStore:
    def __init__(self, environ=None, keyring_loader=_load_keyring):
        self._environ = os.environ if environ is None else environ
        self._keyring_loader = keyring_loader
        self._memory = {}

    def resolve(self, account, env_var=""):
        value = self._environ.get(env_var, "") if env_var else ""
        if value:
            return ResolvedSecret(value, "env")
        value = self._from_keyring(account)
        if value:
            return ResolvedSecret(value, "keyring")
        value = self._memory.get(account, "")
        if value:
            return ResolvedSecret(value, "memory")
        return ResolvedSecret("", "none")

    def set_memory(self, account, value):
        """Keeps a typed key for this run only ('' forgets it)."""
        if value:
            self._memory[account] = value
        else:
            self._memory.pop(account, None)

    def keyring_available(self):
        return self._keyring_loader() is not None

    def save_to_keyring(self, account, value):
        """Stores the key in the system keyring; False when that is not possible."""
        keyring = self._keyring_loader()
        if keyring is None:
            return False
        try:
            keyring.set_password(KEYRING_SERVICE, account, value)
        except Exception:  # keyring backends raise their own error types
            return False
        return True

    def _from_keyring(self, account):
        keyring = self._keyring_loader()
        if keyring is None:
            return ""
        try:
            return keyring.get_password(KEYRING_SERVICE, account) or ""
        except Exception:  # no usable backend (headless Linux, locked keychain...)
            return ""
