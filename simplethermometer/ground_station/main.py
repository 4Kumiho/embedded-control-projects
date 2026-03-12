"""
@file main.py
@brief SimpleThermometer Ground Station GUI

PyQt6 GUI for displaying real-time temperature data.
Reads from serial port in background thread.
Shows temperature value and live matplotlib plot.

Every line is commented.

@author Andrea (ALTEN Training)
@date 2026-03-12
"""

# --- Standard library imports ---

# Import sys for command-line arguments and exit
import sys

# Import queue for communication between threads
import queue

# Import collections for deque (efficient list for storing time-series data)
from collections import deque

# --- Third-party imports ---

# Import PyQt6 for GUI framework
from PyQt6.QtWidgets import (
    # Main application class
    QApplication,

    # Main window class
    QMainWindow,

    # Container for widgets
    QWidget,

    # Vertical layout manager
    QVBoxLayout,

    # Horizontal layout manager
    QHBoxLayout,

    # Label widget for displaying text
    QLabel,

    # Push button widget
    QPushButton,
)

# Import PyQt6 core classes
from PyQt6.QtCore import (
    # Timer for periodic updates
    QTimer,

    # Qt alignment flags
    Qt,
)

# Import PyQt6 GUI utilities
from PyQt6.QtGui import (
    # Font class for text styling
    QFont,
)

# Import matplotlib for plotting
import matplotlib.pyplot as plt

# Import matplotlib backend for embedding in PyQt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

# Import matplotlib Figure class
from matplotlib.figure import Figure

# Import numpy for numerical operations
import numpy as np

# Import local serial reader class
from serial_reader import SerialReader


# ============================================================================
#   TemperaturePlotter CLASS
# ============================================================================

class TemperaturePlotter:
    """
    Matplotlib figure for plotting temperature data over time.

    Maintains a time-series of temperatures and plots them.
    """

    # ========================================================================
    #   __init__ METHOD
    # ========================================================================

    def __init__(self, max_points=120):
        """
        Initialize the temperature plotter.

        @param max_points: Maximum number of data points to display (120 = 60 sec at 500ms)
        """

        # Store maximum number of points to display
        self.max_points = max_points

        # Create deque to store temperature values (FIFO queue with size limit)
        # Deque automatically discards old data when max size is reached
        self.temperatures = deque(maxlen=max_points)

        # Create deque to store time values (in seconds)
        self.times = deque(maxlen=max_points)

        # Counter for current time (incremented each reading)
        self.time_counter = 0

        # Create matplotlib Figure object
        self.figure = Figure(figsize=(8, 4), dpi=100)

        # Create subplot on the figure
        # (1 row, 1 column, 1st subplot)
        self.ax = self.figure.add_subplot(111)

        # Set axis labels
        self.ax.set_xlabel('Time (seconds)')  # X-axis label
        self.ax.set_ylabel('Temperature (°C)')  # Y-axis label
        self.ax.set_title('Temperature Over Time')  # Plot title

        # Set Y-axis limits to expected temperature range
        self.ax.set_ylim([15, 35])

        # Enable grid on the plot
        self.ax.grid(True, alpha=0.3)

    # ========================================================================
    #   add_data METHOD
    # ========================================================================

    def add_data(self, temperature):
        """
        Add a temperature reading to the plot.

        @param temperature: Temperature value in °C (float)
        """

        # Add temperature to deque (oldest value removed if at max size)
        self.temperatures.append(temperature)

        # Add current time to deque
        self.times.append(self.time_counter)

        # Increment time counter for next reading
        # (each reading is 500ms apart in real system)
        self.time_counter += 0.5

    # ========================================================================
    #   update_plot METHOD
    # ========================================================================

    def update_plot(self):
        """
        Redraw the plot with current data.

        Should be called after adding new data.
        """

        # Clear the previous plot
        self.ax.clear()

        # If we have data to plot
        if len(self.temperatures) > 0:
            # Convert deques to lists for plotting
            # (matplotlib expects lists or arrays)
            times_list = list(self.times)
            temps_list = list(self.temperatures)

            # Plot temperature vs time as line
            self.ax.plot(times_list, temps_list, 'b-', linewidth=2, label='Temperature')

            # Add horizontal reference line at 25°C (example setpoint)
            self.ax.axhline(25.0, color='r', linestyle='--', alpha=0.5, label='Setpoint 25°C')

        # Set axis labels
        self.ax.set_xlabel('Time (seconds)')
        self.ax.set_ylabel('Temperature (°C)')
        self.ax.set_title('Temperature Over Time')

        # Set Y-axis limits
        self.ax.set_ylim([15, 35])

        # Enable grid
        self.ax.grid(True, alpha=0.3)

        # Add legend showing line labels
        self.ax.legend()

        # Redraw the figure canvas
        self.figure.canvas.draw()


# ============================================================================
#   SimpleThermometerApp CLASS (Main Window)
# ============================================================================

class SimpleThermometerApp(QMainWindow):
    """
    Main application window for SimpleThermometer.

    Displays temperature value and plot, receives data from serial thread.
    """

    # ========================================================================
    #   __init__ METHOD (Constructor)
    # ========================================================================

    def __init__(self):
        """
        Initialize the SimpleThermometer GUI application.
        """

        # Call parent class constructor
        super().__init__()

        # Set window title
        self.setWindowTitle("SimpleThermometer - Ground Station")

        # Set window size (width x height)
        self.setGeometry(100, 100, 900, 600)

        # Create central widget (main container)
        central_widget = QWidget()

        # Set central widget for the window
        self.setCentralWidget(central_widget)

        # Create main vertical layout
        main_layout = QVBoxLayout()

        # --- Top Section: Status and Value Display ---

        # Create horizontal layout for top section
        top_layout = QHBoxLayout()

        # Create label for status
        self.status_label = QLabel("Status: Waiting for data...")

        # Set font for status label (larger, bold)
        status_font = QFont()
        status_font.setPointSize(12)
        status_font.setBold(True)
        self.status_label.setFont(status_font)

        # Add status label to top layout
        top_layout.addWidget(self.status_label)

        # Create label for temperature display
        self.temp_label = QLabel("Temp: -- °C")

        # Set font for temperature label (large, bold)
        temp_font = QFont()
        temp_font.setPointSize(20)
        temp_font.setBold(True)
        self.temp_label.setFont(temp_font)

        # Align temperature label to the right
        self.temp_label.setAlignment(Qt.AlignmentFlag.AlignRight)

        # Add temperature label to top layout
        top_layout.addWidget(self.temp_label)

        # Add top layout to main layout
        main_layout.addLayout(top_layout)

        # --- Middle Section: Plot ---

        # Create temperature plotter
        self.plotter = TemperaturePlotter()

        # Create matplotlib canvas from the figure
        self.canvas = FigureCanvas(self.plotter.figure)

        # Add canvas to main layout
        main_layout.addWidget(self.canvas)

        # --- Bottom Section: Control Buttons ---

        # Create horizontal layout for buttons
        button_layout = QHBoxLayout()

        # Create "Clear Data" button
        clear_button = QPushButton("Clear Data")

        # Connect button click to clear_data method
        clear_button.clicked.connect(self.clear_data)

        # Add clear button to button layout
        button_layout.addWidget(clear_button)

        # Create "Exit" button
        exit_button = QPushButton("Exit")

        # Connect button click to close application
        exit_button.clicked.connect(self.close)

        # Add exit button to button layout
        button_layout.addWidget(exit_button)

        # Add button layout to main layout
        main_layout.addLayout(button_layout)

        # Set the central widget's layout to main layout
        central_widget.setLayout(main_layout)

        # --- Initialize Serial Reader Thread ---

        # Create SerialReader thread to read from COM3
        self.serial_reader = SerialReader(port='COM3', baudrate=115200)

        # Set thread as daemon (will exit when main thread exits)
        self.serial_reader.daemon = True

        # Start the serial reader thread
        self.serial_reader.start()

        # --- Create Timer for GUI Updates ---

        # Create timer for periodic updates
        self.update_timer = QTimer()

        # Connect timer timeout signal to update_gui method
        self.update_timer.timeout.connect(self.update_gui)

        # Start timer with 200ms interval (5 times per second)
        self.update_timer.start(200)

        # Counter for tracking readings
        self.reading_count = 0

    # ========================================================================
    #   update_gui METHOD (Called by Timer)
    # ========================================================================

    def update_gui(self):
        """
        Update GUI with latest data from serial queue.

        Called periodically by timer (every 200ms).
        """

        # Try to get a temperature from the queue (non-blocking)
        try:
            # Get temperature from queue with 0ms timeout (non-blocking)
            temperature = self.serial_reader.queue.get(timeout=0.0)

            # Add temperature to plotter
            self.plotter.add_data(temperature)

            # Increment reading counter
            self.reading_count += 1

            # Update temperature display label
            self.temp_label.setText(f"Temp: {temperature:.1f} °C")

            # Update status label
            self.status_label.setText(f"Status: Connected ({self.reading_count} readings)")

            # Redraw the plot with new data
            self.plotter.update_plot()

        # If no data available in queue (expected most of the time)
        except queue.Empty:
            # Do nothing, update will happen on next timer tick
            pass

    # ========================================================================
    #   clear_data METHOD (Button Callback)
    # ========================================================================

    def clear_data(self):
        """
        Clear all plotted data and reset counters.

        Called when "Clear Data" button is clicked.
        """

        # Clear the temperature deque
        self.plotter.temperatures.clear()

        # Clear the time deque
        self.plotter.times.clear()

        # Reset time counter to zero
        self.plotter.time_counter = 0

        # Reset reading counter
        self.reading_count = 0

        # Update temperature label
        self.temp_label.setText("Temp: -- °C")

        # Update status label
        self.status_label.setText("Status: Cleared")

        # Redraw empty plot
        self.plotter.update_plot()

    # ========================================================================
    #   closeEvent METHOD (Window Close Handler)
    # ========================================================================

    def closeEvent(self, event):
        """
        Handle window close event.

        Called when user clicks close button (X) on window.
        Stops background threads before exiting.

        @param event: Close event object
        """

        # Print status message
        print("[App] Closing application...")

        # Stop the update timer
        self.update_timer.stop()

        # Signal serial reader thread to stop
        self.serial_reader.stop()

        # Accept the close event (window will close)
        event.accept()


# ============================================================================
#   MAIN ENTRY POINT
# ============================================================================

def main():
    """
    Main entry point for the application.

    Creates Qt application and main window, runs event loop.
    """

    # Print startup message
    print("\n" + "="*50)
    print("SimpleThermometer - Ground Station")
    print("="*50 + "\n")

    # Create QApplication instance (must be before creating any widgets)
    app = QApplication(sys.argv)

    # Create main window
    window = SimpleThermometerApp()

    # Display the window
    window.show()

    # Run the event loop (blocks until window is closed)
    exit_code = app.exec()

    # Print exit message
    print("[App] Exiting...\n")

    # Exit with the Qt application's exit code
    sys.exit(exit_code)


# ============================================================================
#   PYTHON SCRIPT ENTRY POINT
# ============================================================================

# Check if this file is run directly (not imported)
if __name__ == "__main__":
    # Call main() function
    main()
