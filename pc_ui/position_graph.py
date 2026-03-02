"""Real-time position graph using matplotlib."""

import collections
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class PositionGraph:
    def __init__(self, max_points=500):
        self.max_points = max_points
        self.times = collections.deque(maxlen=max_points)
        self.actuals = collections.deque(maxlen=max_points)
        self.targets = collections.deque(maxlen=max_points)
        self.start_time = time.time()

        self.fig, self.ax = plt.subplots(figsize=(10, 5))
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Position (deg)")
        self.ax.set_title("Position Control")
        self.ax.grid(True)

        self.line_actual, = self.ax.plot([], [], "b-", label="Actual", linewidth=1.5)
        self.line_target, = self.ax.plot([], [], "r--", label="Target", linewidth=1.5)
        self.ax.legend(loc="upper left")

        self.anim = None

    def add_point(self, actual_deg, target_deg):
        t = time.time() - self.start_time
        self.times.append(t)
        self.actuals.append(actual_deg)
        self.targets.append(target_deg)

    def _update(self, frame):
        if not self.times:
            return self.line_actual, self.line_target

        times = list(self.times)
        self.line_actual.set_data(times, list(self.actuals))
        self.line_target.set_data(times, list(self.targets))

        self.ax.set_xlim(max(0, times[-1] - 10), times[-1] + 0.5)

        all_vals = list(self.actuals) + list(self.targets)
        if all_vals:
            ymin = min(all_vals) - 5
            ymax = max(all_vals) + 5
            if ymax - ymin < 10:
                mid = (ymin + ymax) / 2
                ymin, ymax = mid - 5, mid + 5
            self.ax.set_ylim(ymin, ymax)

        return self.line_actual, self.line_target

    def start(self):
        self.anim = animation.FuncAnimation(
            self.fig, self._update, interval=50, blit=False, cache_frame_data=False
        )

    def show(self):
        self.start()
        plt.show(block=False)
