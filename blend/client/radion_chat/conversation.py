
import json
from datetime import datetime


def save_conversation(path, profile_name, conversation):
    document = {
        "saved_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "profile": profile_name,
        **conversation,
    }
    with open(path, "w", encoding="utf-8") as file:
        json.dump(document, file, indent=2, ensure_ascii=False)


def default_file_name():
    return datetime.now().strftime("radion_chat_%Y%m%d_%H%M%S.json")
