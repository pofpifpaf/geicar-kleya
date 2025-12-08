import json
import time
from pathlib import Path

json_path = Path("../data/data_test.json")

def load_json():
    with open(json_path, "r") as f:
        return json.load(f)

def save_json(data):
    with open(json_path, "w") as f:
        json.dump(data, f, indent=4)

while True:
    # 1. Load JSON
    data = load_json()

    # 2. Modify variable
    data["speed"] += 1
    data["battery"] += 1

    # 3. Save JSON
    save_json(data)
    time.sleep(1)

    