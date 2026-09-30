import json

from radion_chat.profiles import TEMPLATES, Profile, ProfileStore, config_dir
from radion_chat.secrets import SecretStore, llm_key_account


def deepseek():
    return Profile(name="deepseek", base_url="https://api.deepseek.com", model="deepseek-chat",
                   api_key_env="DEEPSEEK_API_KEY", temperature=0.2, vision=True)


def test_save_and_load_round_trip(tmp_path):
    store = ProfileStore(tmp_path / "cfg" / "profiles.json")
    store.put(deepseek())
    store.put(Profile(name="local", base_url="http://localhost:11434/v1", model="qwen"))
    store.active = "local"
    store.confirm_risky = False
    store.save()

    loaded = ProfileStore(store.path)
    loaded.load()
    assert list(loaded.profiles) == ["deepseek", "local"]
    assert loaded.profiles["deepseek"] == deepseek()
    assert loaded.active == "local"
    assert loaded.confirm_risky is False


def test_file_never_contains_a_key(tmp_path):
    secrets = SecretStore(environ={}, keyring_loader=lambda: None)
    profile = deepseek()
    secrets.set_memory(llm_key_account(profile.name), "sk-very-secret-123")
    store = ProfileStore(tmp_path / "profiles.json")
    store.put(profile)
    store.save()
    text = store.path.read_text()
    assert "sk-very-secret-123" not in text
    assert "DEEPSEEK_API_KEY" in text  # only the variable's name is stored
    assert not any("key" in field and "env" not in field for field in json.loads(text)["profiles"][0])


def test_missing_or_corrupt_file_gives_an_empty_store(tmp_path):
    store = ProfileStore(tmp_path / "nope.json")
    store.load()
    assert store.profiles == {} and store.current() is None

    (tmp_path / "bad.json").write_text("{not json")
    store = ProfileStore(tmp_path / "bad.json")
    store.load()
    assert store.profiles == {} and store.confirm_risky is True


def test_hand_edited_file_is_tolerated(tmp_path):
    path = tmp_path / "profiles.json"
    path.write_text(json.dumps({"active": "ghost", "profiles": [
        {"name": "a", "base_url": "http://x/v1", "model": "m", "max_steps": "lots", "future_field": 1},
        {"base_url": "no name"}, "garbage"]}))
    store = ProfileStore(path)
    store.load()
    assert list(store.profiles) == ["a"]
    assert store.profiles["a"].max_steps == 40  # wrong type: default
    assert store.active == "a"                  # unknown active profile: first one


def test_rename_and_delete_keep_the_active_profile_valid():
    store = ProfileStore("unused.json")
    store.put(deepseek())
    store.put(Profile(name="other"))
    store.active = "deepseek"
    renamed = deepseek()
    renamed.name = "ds"
    store.put(renamed, replacing="deepseek")
    assert list(store.profiles) == ["other", "ds"] and store.active == "ds"
    store.delete("ds")
    assert store.active == "other"
    store.delete("other")
    assert store.active == "" and store.current() is None


def test_validation():
    assert deepseek().problems() == []
    bad = Profile(name=" ", base_url="localhost", model="", max_steps=0)
    assert len(bad.problems()) == 4


def test_templates_carry_no_key_and_are_valid_once_completed():
    for template in TEMPLATES.values():
        template.model = template.model or "some-model"
        assert template.problems() == []
        assert not hasattr(template, "api_key")
    assert TEMPLATES["DeepSeek"].api_key_env == "DEEPSEEK_API_KEY"


def test_config_dir_is_per_user(monkeypatch, tmp_path):
    monkeypatch.setattr("sys.platform", "linux")
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    assert config_dir() == tmp_path / "radion_chat"
