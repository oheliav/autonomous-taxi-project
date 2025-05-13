# utils/helpers.py

import json

def load_driving_graph(path):
    try:
        with open(path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        return {}

def save_driving_graph(graph, path):
    with open(path, 'w') as f:
        json.dump(graph, f, indent=2)
