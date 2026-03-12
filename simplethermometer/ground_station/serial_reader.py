"""
@file serial_reader.py
@brief Serial Port Reader (Threading)

Reads temperature values from serial port in a background thread.
This keeps GUI responsive while waiting for serial data.

Every line is commented.

@author Andrea (ALTEN Training)
@date 2026-03-12
"""

# --- Standard library imports ---

# Import threading module for background thread
import threading

# Import queue module for thread-safe communication
import queue

# Import time module for sleep/delays
import time

# --- Third-party imports ---

# Try to import PySerial library (for real serial communication)
try:
    # Import Serial class from pyserial library
    import serial
    # Flag indicating serial library is available
    PYSERIAL_AVAILABLE = True
except ImportError:
    # If import fails, mark serial as unavailable
    PYSERIAL_AVAILABLE = False


# ============================================================================
#   SerialReader CLASS
# ============================================================================

class SerialReader(threading.Thread):
    """
    Background thread that reads temperature data from serial port.

    Runs continuously in background, parsing "TEMP:XX.X°C" messages
    and putting them in a queue for the GUI to consume.

    Example usage:
        reader = SerialReader(port='COM3', baudrate=115200)
        reader.daemon = True
        reader.start()

        while True:
            try:
                temp = reader.queue.get(timeout=1.0)
                print(f"Temperature: {temp}°C")
            except queue.Empty:
                print("No data received")
    """

    # ========================================================================
    #   __init__ METHOD (Constructor)
    # ========================================================================

    def __init__(self, port='COM3', baudrate=115200):
        """
        Initialize the SerialReader thread.

        @param port: Serial port name (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
        @param baudrate: Baud rate for serial communication (115200 bits/second)
        """

        # Call parent class (threading.Thread) constructor
        super().__init__()

        # Store port name in instance variable
        self.port = port

        # Store baud rate in instance variable
        self.baudrate = baudrate

        # Create a thread-safe queue for communication with main thread
        # Queue holds temperature float values
        self.queue = queue.Queue()

        # Flag to signal thread to stop
        # Set to True when we want the thread to exit
        self.stop_flag = False

        # Variable to hold the serial port object
        # Will be None if connection fails
        self.serial_port = None

        # Variable to hold last successfully read temperature
        # Used for fallback if serial reading fails
        self.last_temperature = 25.0

    # ========================================================================
    #   _connect METHOD (Establish Serial Connection)
    # ========================================================================

    def _connect(self):
        """
        Attempt to open serial port connection.

        If PySerial is not available, use simulated data instead.

        @return: True if connected successfully, False otherwise
        """

        # Print status message
        print(f"[SerialReader] Attempting to connect to {self.port}...")

        # If PySerial library is not available
        if not PYSERIAL_AVAILABLE:
            # Print warning message
            print("[SerialReader] PySerial not installed, using simulated data")

            # Return True anyway (we'll use simulated data)
            return True

        # Try to open serial port connection
        try:
            # Create Serial object and open connection
            self.serial_port = serial.Serial(
                # Port name (COM3, /dev/ttyUSB0, etc.)
                port=self.port,

                # Baud rate (bits per second)
                baudrate=self.baudrate,

                # Timeout for read operations (seconds)
                timeout=1.0
            )

            # Print success message
            print(f"[SerialReader] Connected to {self.port} at {self.baudrate} baud")

            # Return True to indicate successful connection
            return True

        # If connection fails (port not found, already in use, etc.)
        except Exception as e:
            # Print error message with exception details
            print(f"[SerialReader] Connection failed: {e}")

            # Set serial port to None
            self.serial_port = None

            # Return False to indicate failure
            return False

    # ========================================================================
    #   _read_line METHOD (Read One Line from Serial)
    # ========================================================================

    def _read_line(self):
        """
        Read one complete line from serial port.

        Line is terminated by \\r\\n (carriage return + line feed).

        @return: String line (without \\r\\n), or None if failed
        """

        # If serial port is not connected
        if self.serial_port is None:
            # Return None (no data)
            return None

        # Try to read data from serial port
        try:
            # Read bytes from serial port until \\n is found
            # readline() blocks until line terminator found or timeout
            line_bytes = self.serial_port.readline()

            # If no bytes were read (timeout or connection lost)
            if not line_bytes:
                # Return None
                return None

            # Decode bytes to string using UTF-8 encoding
            line_str = line_bytes.decode('utf-8')

            # Remove trailing whitespace (\\r\\n, spaces, etc.)
            line_str = line_str.strip()

            # Return the cleaned string
            return line_str

        # If an error occurs while reading
        except Exception as e:
            # Print error message
            print(f"[SerialReader] Read error: {e}")

            # Return None to indicate error
            return None

    # ========================================================================
    #   _parse_temperature METHOD (Extract Temperature from String)
    # ========================================================================

    def _parse_temperature(self, line):
        """
        Parse temperature from string like "TEMP:22.3°C".

        @param line: String to parse (e.g., "TEMP:22.3")
        @return: Temperature float, or None if parse failed
        """

        # If line is None or empty
        if not line:
            # Return None (cannot parse)
            return None

        # Try to parse the string
        try:
            # Check if line starts with "TEMP:"
            if not line.startswith("TEMP:"):
                # Return None (wrong format)
                return None

            # Extract substring after "TEMP:" prefix
            # "TEMP:22.3°C" → "22.3°C"
            temp_str = line[5:]

            # Remove degree symbol and unit (°C)
            # "22.3°C" → "22.3"
            temp_str = temp_str.replace("°C", "").strip()

            # Convert string to float
            # "22.3" → 22.3
            temperature = float(temp_str)

            # Return the parsed temperature value
            return temperature

        # If any error occurs during parsing
        except Exception as e:
            # Print error message for debugging
            print(f"[SerialReader] Parse error: {e}")

            # Return None to indicate parse failure
            return None

    # ========================================================================
    #   run METHOD (Main Thread Loop)
    # ========================================================================

    def run(self):
        """
        Main method of the background thread.

        This runs continuously, reading from serial and putting
        temperature values in the queue for the GUI.

        Called automatically when thread.start() is invoked.
        """

        # Print status message
        print("[SerialReader] Thread started")

        # Try to connect to serial port
        connected = self._connect()

        # Create a counter for simulated data (if no real serial)
        sim_counter = 0

        # Main loop: run until stop_flag is set to True
        while not self.stop_flag:
            # --- Read data (real or simulated) ---

            # If serial port is connected
            if connected and self.serial_port is not None:
                # Read one line from serial port
                line = self._read_line()

            # If not connected, simulate data
            else:
                # Simulate temperature oscillation with sine wave
                import math

                # Calculate angle for sine wave (complete cycle ~120 iterations)
                angle = (2.0 * 3.14159 * sim_counter) / 120.0

                # Calculate simulated temperature: 25°C ± 5°C oscillation
                simulated_temp = 25.0 + 5.0 * math.sin(angle)

                # Format as string to simulate serial message
                line = f"TEMP:{simulated_temp:.1f}"

                # Increment simulation counter
                sim_counter += 1

            # --- Parse temperature from line ---

            # Extract temperature value from line
            temperature = self._parse_temperature(line)

            # If parsing succeeded
            if temperature is not None:
                # Save as last successful reading
                self.last_temperature = temperature

                # Put temperature in queue (non-blocking put)
                self.queue.put(temperature)

            # --- Sleep before next read ---

            # Sleep for 500ms (0.5 seconds) before next iteration
            # This throttles the reading to match 500ms sensor interval
            time.sleep(0.5)

        # Loop exited (stop_flag was set to True)

        # If serial port is open
        if self.serial_port is not None:
            # Close the serial port connection
            self.serial_port.close()

        # Print status message
        print("[SerialReader] Thread stopped")

    # ========================================================================
    #   stop METHOD (Signal Thread to Stop)
    # ========================================================================

    def stop(self):
        """
        Signal the thread to stop running.

        Should be called before program exit.
        """

        # Set stop flag to True
        self.stop_flag = True

        # Print status message
        print("[SerialReader] Stop signal sent")
