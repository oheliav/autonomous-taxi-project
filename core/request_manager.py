# core/request_manager.py

import random

class RequestManager:
    def __init__(self, spawn_points):
        self.spawn_points = spawn_points

    def generate_request(self):
        a, b = random.sample(self.spawn_points, 2)
        return a.location, b.location
