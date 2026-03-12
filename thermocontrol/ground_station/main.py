"""
@file main.py
@brief ThermoControl Ground Station - PyQt6 GUI with Real-Time Plotting

Main application for monitoring and controlling the temperature system.

Features:
    - Real-time temperature/setpoint/command plotting
    - Live data update (10 Hz from firmware)
    - Setpoint control via slider
    - Telemetry statistics display
    - Error checking and recovery

Architecture:
    Main window (PyQt6)
        ├─ Top: Status display (T, Tset, error, command)
        ├─ Middle: Real-time matplotlib plot
        │   ├─ Temperature line (blue)
        │   ├─ Setpoint line (red, dashed)
        │   └─ Command bar (green)
        └─ Bottom: Control panel
            ├─ Setpoint slider (15-40°C)
            ├─ Auto-scroll toggle
            └─ Status text

Data flow:
    1. TCP socket receives binary packets from firmware
    2. Parser decodes telemetry
    3. Data added to deque (circular buffer, last 60 sec)
    4. Plot refreshes at ~30 Hz (every 33ms)
    5. User adjusts slider → sends command packet to firmware

@author Andrea (ALTEN Training)
@date 2026-03-12
"""

import sys
import time
import socket
import threading
from collections import deque
from typing import Optional, List

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton, QCheckBox, QSpinBox, QTextEdit
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread, QObject
from PyQt6.QtGui import QFont
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np

from parser import (
    parse_telemetry_frame, encode_command, TelemetryPacket,
    format_packet, MAX_FRAME_SIZE
)

# ============================================================================
#   CONSTANTS
# ============================================================================

# Server connection
DEFAULT_HOST = "127.0.0.1"  # Localhost (firmware simulator on same PC)
DEFAULT_PORT = 5555         # Port for telemetry socket

# Plot parameters
MAX_HISTORY = 600           # Keep last 600 seconds (10 minutes)
PLOT_UPDATE_MS = 33         # Refresh plot every 33ms (~30 Hz)
TELEMETRY_RATE_HZ = 10      # Firmware sends 10 telemetry per second


# ============================================================================
#   WORKER THREAD (Network Communication)
# ============================================================================

class TelemetryWorker(QObject):
    """
    Worker thread that receives telemetry from firmware.

    Runs in separate thread to avoid blocking GUI.
    Emits signals when data arrives.

    Communication protocol:
        - Firmware sends telemetry packets every 100ms (10 Hz)
        - Each packet is 21 bytes (fixed size)
        - Uses TCP socket (reliable, ordered delivery)

    Signal flow:
        TelemetryWorker.packet_received → MainWindow.on_packet_received()

    Threading model:
        - Main thread: GUI updates, user input
        - Worker thread: Network I/O (blocking)
        - Signals: Thread-safe communication between them
    """

    # Signals (emitted from worker thread, caught in main thread)
    packet_received = pyqtSignal(TelemetryPacket)
    connection_status_changed = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)

    def __init__(self, host: str, port: int):
        """
        Initialize worker thread.

        Args:
            host: Hostname/IP of firmware (e.g., "127.0.0.1")
            port: TCP port (e.g., 5555)
        """
        super().__init__()
        self.host = host
        self.port = port
        self.running = True
        self.socket: Optional[socket.socket] = None

    def run(self):
        """
        Main worker loop - connect and receive packets.

        Runs in background thread. Handles:
        1. Connect to firmware TCP server
        2. Receive telemetry packets in loop
        3. Parse and emit signals
        4. Reconnect on error

        Does not return until stop() is called.
        """
        while self.running:
            try:
                # Step 1: Connect to firmware
                if self.socket is None:
                    self._connect()

                # Step 2: Receive and parse packets
                self._receive_loop()

            except Exception as e:
                self.error_occurred.emit(f"Connection error: {str(e)}")
                self.connection_status_changed.emit(False)

                # Try to reconnect after 1 second
                if self.running:
                    time.sleep(1.0)

    def _connect(self):
        """
        Establish TCP connection to firmware.

        Blocks until connected or error.
        """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(5.0)  # 5 second timeout

        try:
            self.socket.connect((self.host, self.port))
            self.connection_status_changed.emit(True)
        except socket.timeout:
            raise RuntimeError(f"Timeout connecting to {self.host}:{self.port}")
        except ConnectionRefusedError:
            raise RuntimeError(f"Connection refused by {self.host}:{self.port}")

    def _receive_loop(self):
        """
        Continuous packet receive loop.

        Reads data, looks for complete frames (21 bytes), parses them.
        Handles partial frames by buffering.
        """
        buffer = bytearray()

        while self.running and self.socket:
            try:
                # Receive up to 4KB at once
                data = self.socket.recv(4096)

                if not data:
                    # Connection closed by firmware
                    raise RuntimeError("Connection closed by server")

                # Add to buffer
                buffer.extend(data)

                # Try to extract complete frames from buffer
                while len(buffer) >= MAX_FRAME_SIZE:
                    # Search for magic byte (0x54)
                    try:
                        magic_pos = buffer.index(0x54)
                    except ValueError:
                        # No magic byte in buffer, discard all
                        buffer.clear()
                        break

                    # Check if we have complete frame after magic byte
                    if len(buffer) < magic_pos + MAX_FRAME_SIZE:
                        # Not enough data yet, shift buffer
                        buffer = buffer[magic_pos:]
                        break

                    # Extract frame
                    frame = bytes(buffer[magic_pos : magic_pos + MAX_FRAME_SIZE])

                    # Try to parse
                    packet = parse_telemetry_frame(frame)
                    if packet:
                        # Add timestamp
                        packet.timestamp = time.time()
                        # Emit signal (caught in main thread)
                        self.packet_received.emit(packet)

                    # Move buffer past this frame
                    buffer = buffer[magic_pos + MAX_FRAME_SIZE:]

            except socket.timeout:
                # No data received in 5 seconds, reconnect
                raise RuntimeError("Socket timeout")

    def send_command(self, setpoint: float) -> bool:
        """
        Send setpoint command to firmware.

        Args:
            setpoint: New setpoint temperature (°C)

        Returns:
            True if sent, False if not connected
        """
        if not self.socket:
            return False

        try:
            frame = encode_command(setpoint)
            self.socket.sendall(frame)
            return True
        except Exception as e:
            self.error_occurred.emit(f"Send error: {str(e)}")
            return False

    def stop(self):
        """Stop worker thread"""
        self.running = False
        if self.socket:
            self.socket.close()


# ============================================================================
#   MAIN GUI WINDOW
# ============================================================================

class ThermoControlWindow(QMainWindow):
    """
    Main application window with real-time telemetry display.

    Layout:
        ┌─────────────────────────────────────────┐
        │ Status: T=21.5°C Tset=25.0°C err=3.5°C  │
        ├─────────────────────────────────────────┤
        │                                         │
        │           Real-time Plot                │
        │  Temperature (blue) vs Setpoint (red)   │
        │  Command bar (green)                    │
        │                                         │
        │  Last 60 seconds of data displayed      │
        │                                         │
        ├─────────────────────────────────────────┤
        │ Setpoint: [========•=====] 25.0°C        │
        │ [Send]  [Reset]  [Auto-scroll] [Stat]  │
        └─────────────────────────────────────────┘
    """

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT):
        """
        Initialize main window.

        Args:
            host: Firmware hostname
            port: Firmware telemetry port
        """
        super().__init__()
        self.setWindowTitle("ThermoControl Ground Station")
        self.setGeometry(100, 100, 1200, 700)

        # Data storage (circular buffer, last 600 seconds)
        self.time_data: deque = deque(maxlen=MAX_HISTORY)
        self.temp_data: deque = deque(maxlen=MAX_HISTORY)
        self.setpoint_data: deque = deque(maxlen=MAX_HISTORY)
        self.command_data: deque = deque(maxlen=MAX_HISTORY)

        # Statistics
        self.packet_count = 0
        self.last_packet_time = 0

        # Start telemetry worker thread
        self.worker = TelemetryWorker(host, port)
        self.worker_thread = QThread()
        self.worker.moveToThread(self.worker_thread)

        self.worker_thread.started.connect(self.worker.run)
        self.worker.packet_received.connect(self.on_packet_received)
        self.worker.connection_status_changed.connect(self.on_connection_status)
        self.worker.error_occurred.connect(self.on_error)

        self.worker_thread.start()

        # Build UI
        self._create_ui()

        # Setup plot refresh timer
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(PLOT_UPDATE_MS)

    def _create_ui(self):
        """Create the user interface"""
        # Central widget
        central = QWidget()
        self.setCentralWidget(central)

        layout = QVBoxLayout(central)

        # ========== TOP: Status Display ==========
        status_layout = QHBoxLayout()
        self.status_label = QLabel("Waiting for connection...")
        self.status_label.setFont(QFont("Courier", 11))
        status_layout.addWidget(self.status_label)
        layout.addLayout(status_layout)

        # ========== MIDDLE: Plot ==========
        self.figure = Figure(figsize=(12, 5), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        # ========== BOTTOM: Controls ==========
        control_layout = QHBoxLayout()

        # Setpoint slider (15-40°C)
        control_layout.addWidget(QLabel("Setpoint (°C):"))
        self.setpoint_slider = QSlider(Qt.Orientation.Horizontal)
        self.setpoint_slider.setMinimum(15)
        self.setpoint_slider.setMaximum(40)
        self.setpoint_slider.setValue(25)
        self.setpoint_slider.sliderMoved.connect(self.on_setpoint_changed)
        control_layout.addWidget(self.setpoint_slider, stretch=1)

        self.setpoint_display = QLabel("25.0°C")
        self.setpoint_display.setFont(QFont("Courier", 10))
        control_layout.addWidget(self.setpoint_display)

        # Buttons
        send_btn = QPushButton("Send Command")
        send_btn.clicked.connect(self.on_send_command)
        control_layout.addWidget(send_btn)

        reset_btn = QPushButton("Reset Plot")
        reset_btn.clicked.connect(self.on_reset_plot)
        control_layout.addWidget(reset_btn)

        # Auto-scroll checkbox
        self.autoscroll_check = QCheckBox("Auto-scroll")
        self.autoscroll_check.setChecked(True)
        control_layout.addWidget(self.autoscroll_check)

        layout.addLayout(control_layout)

        # ========== STATISTICS ==========
        self.stats_text = QTextEdit()
        self.stats_text.setReadOnly(True)
        self.stats_text.setMaximumHeight(80)
        self.stats_text.setFont(QFont("Courier", 9))
        layout.addWidget(self.stats_text)

    def on_packet_received(self, packet: TelemetryPacket):
        """
        Handle incoming telemetry packet (slot for signal).

        Called from worker thread via signal (thread-safe).
        Adds data to circular buffers for plotting.

        Args:
            packet: TelemetryPacket decoded from binary
        """
        # Get current time for x-axis
        if not self.time_data:
            t0 = time.time()
            self.start_time = t0
        else:
            t0 = self.start_time

        t_relative = packet.timestamp - t0

        # Add to circular buffers
        self.time_data.append(t_relative)
        self.temp_data.append(packet.temperature)
        self.setpoint_data.append(packet.setpoint)
        self.command_data.append(packet.command)

        # Update statistics
        self.packet_count += 1
        self.last_packet_time = time.time()

        # Update status display
        status_text = (
            f"T={packet.temperature:6.2f}°C | "
            f"Tset={packet.setpoint:6.2f}°C | "
            f"err={packet.error:6.2f}°C | "
            f"cmd={packet.command:6.1f}% | "
            f"packets={self.packet_count}"
        )
        self.status_label.setText(status_text)

        # Update statistics text
        now = time.time()
        time_since_last = now - self.last_packet_time
        if self.packet_count > 0:
            avg_rate = self.packet_count / (now - (self.start_time if hasattr(self, 'start_time') else now))
            stats = (
                f"Packets: {self.packet_count} | "
                f"Avg Rate: {avg_rate:.1f} Hz | "
                f"Last: {time_since_last:.2f}s ago\n"
                f"Temp range: {min(self.temp_data) if self.temp_data else 0:.2f} - {max(self.temp_data) if self.temp_data else 0:.2f}°C"
            )
            self.stats_text.setText(stats)

    def on_connection_status(self, connected: bool):
        """Handle connection status change"""
        status = "✓ Connected" if connected else "✗ Disconnected"
        self.status_label.setText(f"{status} to firmware")

    def on_error(self, error_msg: str):
        """Handle error message from worker"""
        self.stats_text.setText(f"ERROR: {error_msg}")

    def on_setpoint_changed(self, value: int):
        """Handle setpoint slider change"""
        setpoint = float(value)
        self.setpoint_display.setText(f"{setpoint:.1f}°C")

    def on_send_command(self):
        """Send setpoint command to firmware"""
        setpoint = float(self.setpoint_slider.value())
        if self.worker.send_command(setpoint):
            self.stats_text.setText(f"Sent setpoint: {setpoint:.1f}°C")
        else:
            self.stats_text.setText("ERROR: Not connected")

    def on_reset_plot(self):
        """Clear plot data"""
        self.time_data.clear()
        self.temp_data.clear()
        self.setpoint_data.clear()
        self.command_data.clear()
        self.packet_count = 0

    def update_plot(self):
        """Update the real-time plot (called every 33ms)"""
        if not self.temp_data or not self.time_data:
            return

        # Convert deques to lists for plotting
        times = list(self.time_data)
        temps = list(self.temp_data)
        setpoints = list(self.setpoint_data)
        commands = list(self.command_data)

        # Clear previous plot
        self.figure.clear()

        # Create subplots
        ax1 = self.figure.add_subplot(111)

        # Plot temperature and setpoint
        if self.autoscroll_check.isChecked() and len(times) > 0:
            x_min = max(0, times[-1] - 60)  # Show last 60 seconds
            x_max = times[-1]
        else:
            x_min = 0 if times else 0
            x_max = 60

        ax1.plot(times, temps, 'b-', label='Temperature', linewidth=2)
        ax1.plot(times, setpoints, 'r--', label='Setpoint', linewidth=2)
        ax1.set_ylabel('Temperature (°C)', color='r')
        ax1.set_xlabel('Time (seconds)')
        ax1.set_xlim(x_min, x_max)
        ax1.set_ylim(15, 40)
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='upper left')

        # Secondary axis for command
        ax2 = ax1.twinx()
        ax2.bar(times, commands, alpha=0.3, color='g', label='Heater Command', width=0.1)
        ax2.set_ylabel('Command (%)', color='g')
        ax2.set_ylim(-100, 100)
        ax2.legend(loc='upper right')

        self.figure.tight_layout()
        self.canvas.draw_idle()

    def closeEvent(self, event):
        """Handle window close event"""
        self.plot_timer.stop()
        self.worker.stop()
        self.worker_thread.quit()
        self.worker_thread.wait()
        event.accept()


# ============================================================================
#   APPLICATION ENTRY POINT
# ============================================================================

def main():
    """Main entry point - create and run application"""
    app = QApplication(sys.argv)

    # Create main window
    window = ThermoControlWindow(
        host=DEFAULT_HOST,
        port=DEFAULT_PORT
    )
    window.show()

    # Run event loop
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
