import json
from pathlib import Path

CONFIG_PATH = Path("config/email_config.json")

DEFAULT_CONFIG = {
    "sender_email": "",
    "app_password": "",
    "receiver_email": ""
}

def load_email_config():
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)

    if not CONFIG_PATH.exists():
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(DEFAULT_CONFIG, f, indent=4)

        print("Email config file created:")
        print(CONFIG_PATH)
        print("Please fill in sender_email, app_password and receiver_email.")
        return None

    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        config = json.load(f)

    if not config["sender_email"] or not config["app_password"] or not config["receiver_email"]:
        print("Email config file is incomplete.")
        print("Please edit:", CONFIG_PATH)
        return None

    return config
