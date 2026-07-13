class DebounceFilter:
    def __init__(self, hold_time: float = 0.0):
        self.hold_time = hold_time
        self.stable = False
        self.last_raw = False
        self.last_raw_time = 0.0

    def update(self, raw: bool, now: float) -> bool:
        if raw != self.last_raw:
            self.last_raw = raw
            self.last_raw_time = now
        if raw != self.stable and (now - self.last_raw_time) >= self.hold_time:
            self.stable = raw
        return self.stable
