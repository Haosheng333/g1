import math
from collections import deque

class Detector:
    def __init__(self, window=10, threshold=5.0, stable_span=3.0,
                 debounce=0.15, min_hold=1.0, stale=0.25, masked=()):
        if window < 2 or min(threshold, stable_span, debounce, stale) <= 0 or min_hold < 0:
            raise ValueError('Invalid detector parameters')
        self.window, self.threshold, self.stable_span = window, threshold, stable_span
        self.debounce, self.min_hold, self.stale = debounce, min_hold, stale
        self.masked = set(masked)
        self.disarm()

    def disarm(self):
        self.state = 'IDLE'
        self.frames = deque(maxlen=self.window)
        self.baseline = None
        self.since = {}
        self.last = None

    def arm(self, now):
        self.disarm()
        self.started = now
        self.state = 'BASELINING'

    def feed(self, values, now):
        if self.state in ('IDLE', 'TRIGGERED', 'FAULT'):
            return False
        try:
            v = [float(x) for x in values]
            valid = bool(v) and all(math.isfinite(x) for x in v)
        except (ValueError, TypeError):
            valid = False
        if not valid:
            self.state = 'FAULT'
            return False
        if self.last is not None and (now < self.last or now-self.last > self.stale):
            self.state = 'FAULT'
            return False
        if self.frames and len(v) != len(self.frames[0]):
            self.state = 'FAULT'
            return False
        self.last = now
        self.frames.append(v)
        active = [i for i in range(len(v)) if i not in self.masked]
        if not active:
            self.state = 'FAULT'
            return False
        if len(self.frames) < self.window:
            return False
        avg = [sum(f[i] for f in self.frames)/self.window for i in range(len(v))]
        if self.state == 'BASELINING':
            if all(max(f[i] for f in self.frames)-min(f[i] for f in self.frames) <= self.stable_span for i in active):
                self.baseline = avg
                self.state = 'MONITORING'
            return False
        if now-self.started < self.min_hold:
            self.since.clear()
            return False
        for i in active:
            if abs(avg[i]-self.baseline[i]) <= self.threshold:
                self.baseline[i] += 0.05 * (avg[i]-self.baseline[i])
                self.since.pop(i, None)
            else:
                self.since.setdefault(i, now)
                if now-self.since[i] >= self.debounce:
                    self.state = 'TRIGGERED'
                    return True
        return False

    def check_stale(self, now):
        if self.state in ('BASELINING', 'MONITORING'):
            if now-(self.last if self.last is not None else self.started) > self.stale:
                self.state = 'FAULT'
        return self.state
