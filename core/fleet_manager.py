# core/fleet_manager.py

class FleetManager:
    def __init__(self, vehicles):
        self.taxis = {v.id: v for v in vehicles}
        self.available = set(self.taxis.keys())

    def get_available_taxis(self):
        return [self.taxis[tid] for tid in self.available]

    def mark_taxi_unavailable(self, taxi_id):
        self.available.discard(taxi_id)

    def mark_taxi_available(self, taxi_id):
        self.available.add(taxi_id)
