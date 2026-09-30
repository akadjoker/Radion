from radion_chat.secrets import KEYRING_SERVICE, SecretStore


class FakeKeyring:
    def __init__(self, stored=None, broken=False):
        self.stored = dict(stored or {})
        self.broken = broken

    def get_password(self, service, account):
        if self.broken:
            raise RuntimeError("no backend")
        return self.stored.get((service, account))

    def set_password(self, service, account, value):
        if self.broken:
            raise RuntimeError("no backend")
        self.stored[(service, account)] = value


def store(environ=None, keyring=None):
    return SecretStore(environ=environ or {}, keyring_loader=lambda: keyring)


def test_nothing_anywhere():
    resolved = store().resolve("llm:a", "DEEPSEEK_API_KEY")
    assert (resolved.value, resolved.source) == ("", "none")


def test_environment_beats_keyring_and_memory():
    keyring = FakeKeyring({(KEYRING_SERVICE, "llm:a"): "from-keyring"})
    secrets = store({"DEEPSEEK_API_KEY": "from-env"}, keyring)
    secrets.set_memory("llm:a", "from-memory")
    resolved = secrets.resolve("llm:a", "DEEPSEEK_API_KEY")
    assert (resolved.value, resolved.source) == ("from-env", "env")


def test_keyring_beats_memory():
    keyring = FakeKeyring({(KEYRING_SERVICE, "llm:a"): "from-keyring"})
    secrets = store({}, keyring)
    secrets.set_memory("llm:a", "from-memory")
    resolved = secrets.resolve("llm:a", "DEEPSEEK_API_KEY")
    assert (resolved.value, resolved.source) == ("from-keyring", "keyring")


def test_memory_is_the_last_resort_and_can_be_forgotten():
    secrets = store()
    secrets.set_memory("llm:a", "typed")
    assert secrets.resolve("llm:a").source == "memory"
    secrets.set_memory("llm:a", "")
    assert secrets.resolve("llm:a").source == "none"


def test_an_unset_or_empty_variable_falls_through():
    secrets = store({"EMPTY": ""})
    secrets.set_memory("llm:a", "typed")
    assert secrets.resolve("llm:a", "EMPTY").source == "memory"
    assert secrets.resolve("llm:a", "NOT_SET").source == "memory"


def test_keys_are_per_account():
    secrets = store()
    secrets.set_memory("llm:a", "one")
    assert secrets.resolve("llm:b").source == "none"


def test_keyring_not_installed_is_fine():
    secrets = store(keyring=None)
    assert not secrets.keyring_available()
    assert secrets.save_to_keyring("llm:a", "x") is False
    assert secrets.resolve("llm:a").source == "none"


def test_broken_keyring_backend_is_ignored():
    secrets = store(keyring=FakeKeyring(broken=True))
    secrets.set_memory("llm:a", "typed")
    assert secrets.resolve("llm:a").source == "memory"
    assert secrets.save_to_keyring("llm:a", "x") is False


def test_save_to_keyring_then_resolve():
    keyring = FakeKeyring()
    secrets = store(keyring=keyring)
    assert secrets.save_to_keyring("llm:a", "kept")
    assert keyring.stored == {(KEYRING_SERVICE, "llm:a"): "kept"}
    assert secrets.resolve("llm:a").value == "kept"


def test_real_loader_survives_a_missing_keyring_package(monkeypatch):
    import sys
    monkeypatch.setitem(sys.modules, "keyring", None)
    assert not SecretStore(environ={}).keyring_available()
