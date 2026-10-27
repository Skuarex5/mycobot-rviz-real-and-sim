"""Destination ownership and plan provenance, independent of ROS transport."""

import math


class SelectionPolicy:
    def __init__(self):
        self.physical = self.live = False
        self.generation = self.jobs = 0
        self.plans = set()
        self.available = True

    def select(self, physical):
        if self.jobs or self.live:
            raise ValueError(
                "Detené el seguimiento o la ejecución antes de cambiar destino"
            )
        if self.physical != physical:
            self.generation += 1
            self.plans.clear()
        self.physical = physical
        self.available = True

    def begin_job(self):
        if not self.available or self.live:
            raise ValueError("Destino no disponible o seguimiento activo")
        self.jobs += 1
        return ("physical" if self.physical else "simulation", self.generation)

    def end_job(self):
        self.jobs = max(0, self.jobs - 1)

    def record_plan(self, signature, generation):
        if generation == self.generation:
            self.plans.add(signature)

    def valid_plan(self, signature):
        return signature in self.plans

    def lost(self):
        self.available = False
        self.generation += 1
        self.plans.clear()


class StateSource:
    def __init__(self, names, timeout=0.5):
        self.names = tuple(names)
        self.timeout = timeout
        self.stamp = self.received = None
        self.positions = None

    def update(self, names, positions, stamp, now):
        if (
            len(names) != len(positions)
            or len(set(names)) != len(names)
            or not math.isfinite(stamp)
            or stamp == self.stamp
        ):
            return False
        values = dict(zip(names, positions))
        if any(n not in values or not math.isfinite(values[n]) for n in self.names):
            return False
        self.positions = [values[n] for n in self.names]
        self.stamp, self.received = stamp, now
        return True

    def fresh(self, now):
        return self.received is not None and now - self.received <= self.timeout
