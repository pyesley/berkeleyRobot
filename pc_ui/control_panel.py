"""Control panel GUI with position slider, PID tuning, and motor controls."""

import tkinter as tk
from tkinter import ttk


class ControlPanel:
    def __init__(self, on_position=None, on_pid=None, on_command=None):
        self.on_position = on_position
        self.on_pid = on_pid
        self.on_command = on_command

        self.root = tk.Tk()
        self.root.title("Berkeley Robot - Position Control")
        self.root.geometry("500x650")

        self._build_ui()

    def _build_ui(self):
        # Position control
        frame_pos = ttk.LabelFrame(self.root, text="Position Control", padding=10)
        frame_pos.pack(fill="x", padx=10, pady=5)

        ttk.Label(frame_pos, text="Target Position (deg):").pack(anchor="w")
        self.pos_var = tk.DoubleVar(value=0.0)
        self.pos_slider = ttk.Scale(
            frame_pos, from_=-180, to=180, variable=self.pos_var,
            orient="horizontal", length=400,
            command=self._on_slider_change,
        )
        self.pos_slider.pack(fill="x")

        pos_entry_frame = ttk.Frame(frame_pos)
        pos_entry_frame.pack(fill="x", pady=5)
        self.pos_entry = ttk.Entry(pos_entry_frame, width=10)
        self.pos_entry.insert(0, "0.0")
        self.pos_entry.pack(side="left")
        ttk.Button(pos_entry_frame, text="Go", command=self._on_go).pack(side="left", padx=5)

        # PID tuning
        frame_pid = ttk.LabelFrame(self.root, text="PID Gains", padding=10)
        frame_pid.pack(fill="x", padx=10, pady=5)

        pid_grid = ttk.Frame(frame_pid)
        pid_grid.pack(fill="x")

        ttk.Label(pid_grid, text="Kp:").grid(row=0, column=0, sticky="w")
        self.kp_var = tk.StringVar(value="10.0")
        ttk.Entry(pid_grid, textvariable=self.kp_var, width=10).grid(row=0, column=1, padx=5)

        ttk.Label(pid_grid, text="Ki:").grid(row=1, column=0, sticky="w")
        self.ki_var = tk.StringVar(value="1.0")
        ttk.Entry(pid_grid, textvariable=self.ki_var, width=10).grid(row=1, column=1, padx=5)

        ttk.Button(frame_pid, text="Apply PID", command=self._on_apply_pid).pack(pady=5)

        # Motor commands
        frame_cmd = ttk.LabelFrame(self.root, text="Motor Commands", padding=10)
        frame_cmd.pack(fill="x", padx=10, pady=5)

        btn_frame = ttk.Frame(frame_cmd)
        btn_frame.pack()

        ttk.Button(btn_frame, text="Enable", command=lambda: self._send_cmd("enable")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Disable", command=lambda: self._send_cmd("disable")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Home", command=lambda: self._send_cmd("home")).pack(side="left", padx=5)

        estop_btn = tk.Button(
            btn_frame, text="E-STOP", command=lambda: self._send_cmd("estop"),
            bg="red", fg="white", font=("Arial", 12, "bold"),
            width=8, height=1,
        )
        estop_btn.pack(side="left", padx=10)

        # Status display
        frame_status = ttk.LabelFrame(self.root, text="Status", padding=10)
        frame_status.pack(fill="x", padx=10, pady=5)

        self.status_text = tk.Text(frame_status, height=8, width=55, state="disabled",
                                   font=("Consolas", 10))
        self.status_text.pack()

    def _on_slider_change(self, val):
        deg = float(val)
        self.pos_entry.delete(0, "end")
        self.pos_entry.insert(0, f"{deg:.1f}")
        if self.on_position:
            self.on_position(deg)

    def _on_go(self):
        try:
            deg = float(self.pos_entry.get())
            self.pos_var.set(deg)
            if self.on_position:
                self.on_position(deg)
        except ValueError:
            pass

    def _on_apply_pid(self):
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            if self.on_pid:
                self.on_pid(kp, ki)
        except ValueError:
            pass

    def _send_cmd(self, cmd):
        if self.on_command:
            self.on_command(cmd)

    def update_status(self, info: dict):
        self.status_text.config(state="normal")
        self.status_text.delete("1.0", "end")
        for key, val in info.items():
            self.status_text.insert("end", f"{key:>15s}: {val}\n")
        self.status_text.config(state="disabled")

    def run(self):
        self.root.mainloop()
