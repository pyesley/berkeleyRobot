"""
Berkeley Robot Position Control - PC UI Application

Usage:
    python main.py [--interface pcan] [--channel PCAN_USBBUS1] [--bitrate 1000000]

Requires: python-can, matplotlib, tkinter
    pip install python-can matplotlib
"""

import argparse
import sys
import threading
import time

import can

from can_protocol import (
    CAN_ID_TELEMETRY, CAN_ID_STATUS,
    CMD_ENABLE, CMD_DISABLE, CMD_HOME, CMD_ESTOP,
    encode_set_position, encode_set_pid_gains, encode_command,
    decode_telemetry, decode_status,
)
from position_graph import PositionGraph
from control_panel import ControlPanel


class PositionControlApp:
    def __init__(self, interface="pcan", channel="PCAN_USBBUS1", bitrate=1000000):
        self.bus = None
        self.running = False
        self.last_status = {}
        self.last_telemetry = {}

        # Try to connect CAN bus
        try:
            self.bus = can.Bus(interface=interface, channel=channel, bitrate=bitrate)
            print(f"CAN bus connected: {interface} / {channel} @ {bitrate} bps")
        except Exception as e:
            print(f"WARNING: CAN bus not available ({e})")
            print("Running in offline mode - no CAN communication")

        # Create graph
        self.graph = PositionGraph()

        # Create control panel with callbacks
        self.panel = ControlPanel(
            on_position=self._on_position,
            on_pid=self._on_pid,
            on_command=self._on_command,
        )

    def _send_can(self, arb_id, data):
        if self.bus is None:
            return
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"CAN send error: {e}")

    def _on_position(self, deg):
        arb_id, data = encode_set_position(deg)
        self._send_can(arb_id, data)

    def _on_pid(self, kp, ki):
        arb_id, data = encode_set_pid_gains(kp, ki)
        self._send_can(arb_id, data)

    def _on_command(self, cmd_name):
        cmd_map = {
            "enable": CMD_ENABLE,
            "disable": CMD_DISABLE,
            "home": CMD_HOME,
            "estop": CMD_ESTOP,
        }
        cmd = cmd_map.get(cmd_name)
        if cmd is not None:
            arb_id, data = encode_command(cmd)
            self._send_can(arb_id, data)

    def _rx_thread(self):
        while self.running:
            if self.bus is None:
                time.sleep(0.1)
                continue

            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue

            if msg.arbitration_id == CAN_ID_TELEMETRY:
                self.last_telemetry = decode_telemetry(msg.data)
                self.graph.add_point(
                    self.last_telemetry["actual_deg"],
                    self.last_telemetry["target_deg"],
                )

            elif msg.arbitration_id == CAN_ID_STATUS:
                self.last_status = decode_status(msg.data)
                status_info = {**self.last_telemetry, **self.last_status}
                try:
                    self.panel.root.after(0, self.panel.update_status, status_info)
                except Exception:
                    pass

    def run(self):
        self.running = True

        # Start CAN RX thread
        rx = threading.Thread(target=self._rx_thread, daemon=True)
        rx.start()

        # Show graph (non-blocking)
        self.graph.show()

        # Run control panel (blocking - tkinter main loop)
        try:
            self.panel.run()
        finally:
            self.running = False
            if self.bus:
                self.bus.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Berkeley Robot Position Control UI")
    parser.add_argument("--interface", default="pcan",
                        help="python-can interface (pcan, socketcan, gs_usb, etc.)")
    parser.add_argument("--channel", default="PCAN_USBBUS1",
                        help="CAN channel name")
    parser.add_argument("--bitrate", type=int, default=1000000,
                        help="CAN bus bitrate (default 1000000)")
    args = parser.parse_args()

    app = PositionControlApp(
        interface=args.interface,
        channel=args.channel,
        bitrate=args.bitrate,
    )
    app.run()


if __name__ == "__main__":
    main()
