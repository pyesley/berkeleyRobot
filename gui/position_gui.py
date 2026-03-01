#!/usr/bin/env python3
"""
Position Control GUI — Open-Loop Motor Controller

Spawns ./build/open_loop_test goto <deg> [torque] --json as a subprocess,
reads JSON lines from stdout in a thread, and plots position vs time in real-time.
"""

import tkinter as tk
from tkinter import ttk
import json
import threading
import subprocess
import os
import sys
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


# Path to the open_loop_test binary (relative to this script's directory)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
BINARY = os.path.join(PROJECT_DIR, "build", "open_loop_test")


class PositionControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Position Controller — Open Loop")
        self.root.geometry("1400x900")
        self.root.minsize(1000, 700)

        # Subprocess state
        self.process = None
        self.reader_thread = None
        self.moving = False

        # Plot data (accumulated by reader thread, consumed by UI poll)
        self.lock = threading.Lock()
        self.times = []
        self.positions = []
        self.target_deg = 0.0
        self.result = None  # final result dict when move completes
        self.new_data = False  # flag for UI poll

        # Log buffer
        self.max_log_lines = 500

        self.setup_ui()
        self.poll_id = self.root.after(50, self.poll_data)

    def setup_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)  # plot gets most space
        self.root.rowconfigure(2, weight=0)  # controls
        self.root.rowconfigure(3, weight=0)  # log

        # ---- Top bar: status ----
        status_frame = tk.Frame(self.root, bg="#2d2d2d", height=48)
        status_frame.grid(row=0, column=0, sticky="ew")
        status_frame.columnconfigure(1, weight=1)

        self.status_label = tk.Label(
            status_frame, text="Idle", font=("Helvetica", 22, "bold"),
            fg="#aaaaaa", bg="#2d2d2d", padx=20, pady=8)
        self.status_label.grid(row=0, column=0, sticky="w")

        self.result_label = tk.Label(
            status_frame, text="", font=("Helvetica", 18),
            fg="#cccccc", bg="#2d2d2d", padx=20, pady=8)
        self.result_label.grid(row=0, column=1, sticky="w")

        # ---- Middle: matplotlib plot ----
        plot_frame = tk.Frame(self.root)
        plot_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=(5, 0))
        plot_frame.columnconfigure(0, weight=1)
        plot_frame.rowconfigure(0, weight=1)

        self.fig = Figure(figsize=(12, 5), dpi=100, facecolor="#1e1e1e")
        self.ax = self.fig.add_subplot(111)
        self.setup_plot()

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        # ---- Bottom controls ----
        controls_frame = tk.Frame(self.root, bg="#333333", padx=16, pady=12)
        controls_frame.grid(row=2, column=0, sticky="ew")

        # Move-by entry
        tk.Label(controls_frame, text="Move by:", font=("Helvetica", 22),
                 fg="white", bg="#333333").pack(side="left")
        self.deg_entry = tk.Entry(controls_frame, font=("Helvetica", 22), width=8)
        self.deg_entry.insert(0, "90")
        self.deg_entry.pack(side="left", padx=(8, 4))
        tk.Label(controls_frame, text="deg", font=("Helvetica", 22),
                 fg="white", bg="#333333").pack(side="left", padx=(0, 20))

        # Torque entry
        tk.Label(controls_frame, text="Torque:", font=("Helvetica", 22),
                 fg="white", bg="#333333").pack(side="left")
        self.torque_entry = tk.Entry(controls_frame, font=("Helvetica", 22), width=6)
        self.torque_entry.insert(0, "0.05")
        self.torque_entry.pack(side="left", padx=(8, 4))
        tk.Label(controls_frame, text="Nm", font=("Helvetica", 22),
                 fg="white", bg="#333333").pack(side="left", padx=(0, 20))

        # Go button
        self.go_button = tk.Button(
            controls_frame, text="Go", font=("Helvetica", 22, "bold"),
            bg="#4CAF50", fg="white", activebackground="#45a049",
            width=6, command=self.on_go)
        self.go_button.pack(side="left", padx=(10, 30))
        self.deg_entry.bind("<Return>", lambda e: self.on_go())

        # Quick buttons
        tk.Label(controls_frame, text="Quick:", font=("Helvetica", 18),
                 fg="#aaaaaa", bg="#333333").pack(side="left", padx=(0, 8))
        for deg in [-180, -90, -45, 45, 90, 180]:
            sign = "+" if deg > 0 else ""
            btn = tk.Button(
                controls_frame, text=f"{sign}{deg}\u00b0",
                font=("Helvetica", 18), width=5,
                command=lambda d=deg: self.quick_move(d))
            btn.pack(side="left", padx=3)

        # ---- Log panel ----
        log_frame = tk.LabelFrame(self.root, text="Log", font=("Helvetica", 12),
                                  padx=6, pady=4)
        log_frame.grid(row=3, column=0, sticky="ew", padx=10, pady=(5, 10))
        log_frame.columnconfigure(0, weight=1)

        self.log_text = tk.Text(
            log_frame, height=6, wrap="word",
            font=("Courier", 11), state="disabled",
            bg="#1e1e1e", fg="#d4d4d4",
            insertbackground="#d4d4d4", selectbackground="#264f78")
        self.log_text.grid(row=0, column=0, sticky="ew")

        log_scroll = ttk.Scrollbar(log_frame, orient="vertical",
                                   command=self.log_text.yview)
        log_scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.config(yscrollcommand=log_scroll.set)

        btn_frame = tk.Frame(log_frame)
        btn_frame.grid(row=1, column=0, columnspan=2, sticky="e", pady=(2, 0))
        tk.Button(btn_frame, text="Clear Log",
                  command=self.clear_log).pack(side="right")

    def setup_plot(self):
        """Configure the matplotlib axes."""
        self.ax.set_facecolor("#1e1e1e")
        self.ax.set_xlabel("Time (s)", fontsize=16, color="white")
        self.ax.set_ylabel("Output Position (deg)", fontsize=16, color="white")
        self.ax.tick_params(colors="white", labelsize=12)
        for spine in self.ax.spines.values():
            spine.set_color("#555555")
        self.ax.grid(True, color="#333333", linewidth=0.5)
        self.position_line, = self.ax.plot([], [], "dodgerblue", linewidth=2,
                                           label="Position")
        # Draw target as a plain line (not axhline) so it doesn't confuse autoscale
        self.target_line, = self.ax.plot([], [], "r--", linewidth=1.5,
                                         label="Target")
        self.ax.legend(loc="upper left", fontsize=12, facecolor="#2d2d2d",
                       edgecolor="#555555", labelcolor="white")
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(-5, 5)
        self.fig.tight_layout()

    def log(self, message):
        """Append a message to the log panel."""
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"[{ts}] {message}\n")
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > self.max_log_lines:
            self.log_text.delete("1.0", f"{line_count - self.max_log_lines}.0")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def clear_log(self):
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")

    def quick_move(self, degrees):
        self.deg_entry.delete(0, tk.END)
        self.deg_entry.insert(0, str(degrees))
        self.on_go()

    def on_go(self):
        """Start a move by spawning the open_loop_test subprocess."""
        if self.moving:
            return

        try:
            deg = float(self.deg_entry.get())
        except ValueError:
            self.log("Invalid degree value")
            return
        try:
            torque = float(self.torque_entry.get())
        except ValueError:
            self.log("Invalid torque value")
            return

        if not os.path.isfile(BINARY):
            self.log(f"Binary not found: {BINARY}")
            self.log("Run: cd build && cmake --build . --target open_loop_test")
            return

        # Clear plot data
        with self.lock:
            self.times.clear()
            self.positions.clear()
            self.target_deg = deg
            self.result = None
            self.new_data = True

        # Clear plot and set initial view around target
        self.position_line.set_data([], [])
        self.target_line.set_data([0, 1], [deg, deg])
        margin = max(abs(deg) * 0.15, 5.0)
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(min(0, deg) - margin, max(0, deg) + margin)
        self.canvas.draw_idle()

        # Update UI state
        self.moving = True
        self.go_button.config(state="disabled", bg="#888888")
        self.status_label.config(text="Moving...", fg="#FFD700")
        self.result_label.config(text=f"target: {deg:.1f}\u00b0  torque: {torque}")

        # Build command
        cmd = [BINARY, "goto", str(deg)]
        if torque != 0.05:
            cmd.append(str(torque))
        cmd.append("--json")

        self.log(f"CMD: {' '.join(cmd)}")

        try:
            self.process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                bufsize=1, text=True)
        except Exception as e:
            self.log(f"Failed to start process: {e}")
            self.moving = False
            self.go_button.config(state="normal", bg="#4CAF50")
            self.status_label.config(text="Error", fg="red")
            return

        self.reader_thread = threading.Thread(
            target=self.read_stdout, daemon=True)
        self.reader_thread.start()

    def read_stdout(self):
        """Reader thread: parse JSON lines from subprocess stdout."""
        proc = self.process
        try:
            for line in proc.stdout:
                line = line.strip()
                if not line:
                    continue
                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    # Non-JSON line (e.g. stderr leak) — log it
                    self.root.after(0, self.log, f"[non-json] {line}")
                    continue

                if data.get("type") == "result":
                    with self.lock:
                        self.result = data
                        self.new_data = True
                else:
                    with self.lock:
                        self.times.append(data.get("t", 0))
                        self.positions.append(data.get("out", 0))
                        self.new_data = True

                # Also log raw line
                self.root.after(0, self.log, line)

        except Exception as e:
            self.root.after(0, self.log, f"Reader error: {e}")

        # Read stderr
        try:
            stderr = proc.stderr.read()
            if stderr.strip():
                self.root.after(0, self.log, f"[stderr] {stderr.strip()}")
        except:
            pass

        proc.wait()
        self.root.after(0, self.on_process_done, proc.returncode)

    def on_process_done(self, returncode):
        """Called on main thread when subprocess exits."""
        self.moving = False
        self.process = None
        self.go_button.config(state="normal", bg="#4CAF50")

        with self.lock:
            result = self.result

        if result:
            out = result.get("out", 0)
            target = result.get("target", 0)
            error = result.get("error", 0)
            nudges = result.get("nudges", 0)
            t = result.get("time", 0)
            self.status_label.config(text="Done", fg="#4CAF50")
            self.result_label.config(
                text=f"out: {out:.2f}\u00b0  error: {error:+.2f}\u00b0  "
                     f"nudges: {nudges}  time: {t:.1f}s")
        else:
            self.status_label.config(
                text=f"Exited ({returncode})",
                fg="red" if returncode != 0 else "#4CAF50")

    def poll_data(self):
        """Poll at ~20 Hz to update the plot from accumulated data."""
        with self.lock:
            if self.new_data and self.times:
                t = list(self.times)
                p = list(self.positions)
                target = self.target_deg
                self.new_data = False
            else:
                t = p = target = None
                self.new_data = False

        if t is not None:
            # Update position line
            self.position_line.set_data(t, p)

            # Update target line to span the full x range
            t_max = max(t[-1], 0.5)
            self.target_line.set_data([0, t_max], [target, target])

            # Compute y-axis from actual data + target
            all_y = p + [target, 0.0]
            ymin = min(all_y)
            ymax = max(all_y)
            margin = max((ymax - ymin) * 0.15, 5.0)
            self.ax.set_ylim(ymin - margin, ymax + margin)

            # X-axis: 0 to slightly past latest time
            self.ax.set_xlim(0, t_max * 1.05)

            self.canvas.draw_idle()

        self.poll_id = self.root.after(50, self.poll_data)

    def on_close(self):
        """Clean up on window close."""
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
        if self.poll_id:
            self.root.after_cancel(self.poll_id)
        self.root.destroy()


def main():
    root = tk.Tk()
    app = PositionControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
