# Fase 3 — Ground Station (Python + PyQt6)

**Durata:** ~1 ora
**Competenze coperte:** SW OOP (Python), GUI (PyQt6), Threading, Data Visualization

---

## Panoramica

Fase 3 implementa la stazione di terra con:
1. Serial reader thread (background)
2. GUI con PyQt6
3. Real-time plot con matplotlib

**Novità:** Ogni riga commentata per spiegare cosa fa.

---

## Competenza ALTEN Coperta

✅ **SW OOP (Python)** — Classes, threading
✅ **GUI Framework** — PyQt6 widgets
✅ **Data Visualization** — matplotlib real-time plotting

---

## Architettura

```
┌─────────────────────────────────────┐
│ Main Thread (GUI)                   │
│ ┌─────────────────────────────────┐ │
│ │ PyQt6 Window                    │ │
│ │ - QLabel (Temperature display)  │ │
│ │ - Matplotlib Canvas (Plot)      │ │
│ │ - Buttons (Clear, Exit)         │ │
│ └────────┬────────────────────────┘ │
│          │ QTimer (200ms)           │
│          ▼                          │
│     update_gui()                    │
│         │                           │
│         ▼                           │
│   Read from queue  ◄────────┐       │
└────────────────────────────┼────────┘
                             │
                      ┌──────┴────────┐
                      │               │
┌─────────────────────▼───────────────┐
│ Background Thread (Serial Reader)   │
│ ┌─────────────────────────────────┐ │
│ │ SerialReader (threading.Thread) │ │
│ │ - Read from serial              │ │
│ │ - Parse "TEMP:XX.X"             │ │
│ │ - Put in queue                  │ │
│ │ - 500ms loop                    │ │
│ └─────────────────────────────────┘ │
└─────────────────────────────────────┘
```

---

## File Creati

### 1. serial_reader.py — Background Thread

**Classe:** `SerialReader(threading.Thread)`

**Cosa fa:**
- Runs in background thread
- Reads from serial port (COM3)
- Parses "TEMP:XX.X°C" messages
- Puts temperature in thread-safe queue

**Metodi:**

```python
def __init__(self, port='COM3', baudrate=115200):
    # Initialize thread, create queue, store port/baud

def _connect(self):
    # Try to open serial port, fallback to simulated data

def _read_line(self):
    # Read one line from serial (blocking with timeout)

def _parse_temperature(self, line):
    # Extract temperature from "TEMP:22.3°C"

def run(self):
    # Main thread loop (reads and puts data in queue)

def stop(self):
    # Signal thread to stop
```

**Key Features:**
- Thread-safe queue for GUI communication
- Automatic simulation if serial not available
- Error handling for disconnections
- Clean shutdown

**Esempio uso:**
```python
reader = SerialReader(port='COM3')
reader.daemon = True
reader.start()

# In main thread:
try:
    temp = reader.queue.get(timeout=1.0)  # Non-blocking get
    print(f"Temperature: {temp}°C")
except queue.Empty:
    print("No data")
```

### 2. main.py — GUI Application

**Classe:** `SimpleThermometerApp(QMainWindow)`

**Layout:**
```
╔════════════════════════════════════╗
║ Status: Connected  | Temp: 22.3°C  ║  ← Top row (info)
├────────────────────────────────────┤
║                                    ║
║        Temperature Plot            ║
║   (matplotlib canvas)              ║
║                                    ║
├────────────────────────────────────┤
║ [Clear Data]              [Exit]   ║  ← Buttons
╚════════════════════════════════════╝
```

**Componenti:**

```python
class SimpleThermometerApp(QMainWindow):

    def __init__(self):
        # Setup window, widgets, layouts

    def update_gui(self):
        # Called by timer every 200ms
        # Get temperature from queue
        # Update label and plot

    def clear_data(self):
        # Clear plot data

    def closeEvent(self, event):
        # Stop threads before exit
```

**Key Widgets:**
- `QLabel` — Status e temperature display
- `FigureCanvas` — Matplotlib plot
- `QPushButton` — Clear, Exit buttons

**Fonts & Styling:**
```python
# Temperature label: size 20, bold
temp_font = QFont()
temp_font.setPointSize(20)
temp_font.setBold(True)
self.temp_label.setFont(temp_font)
```

### 3. TemperaturePlotter — Matplotlib Integration

**Classe:** `TemperaturePlotter`

**Cosa fa:**
- Maintains deque of temperatures (max 120 points = 60 sec)
- Creates matplotlib Figure
- Updates plot with new data

**Metodi:**
```python
def __init__(self, max_points=120):
    # Setup figure, axes, deques

def add_data(self, temperature):
    # Append to temperature deque

def update_plot(self):
    # Redraw plot with current data
```

**Plot Features:**
- X-axis: Time (seconds)
- Y-axis: Temperature (15-35°C)
- Blue line: Temperature trace
- Red dashed line: Setpoint (25°C)
- Grid enabled

---

## Threading Model

```
Main Thread:
  - PyQt6 event loop
  - User interactions
  - Timer-driven updates
  - GUI rendering

Background Thread (SerialReader):
  - Blocks on serial.read()
  - Parses incoming data
  - Non-blocking queue.put()
  - Independent timing
```

**Synchronization:**
- `queue.Queue()` for thread-safe communication
- Main thread uses `queue.get(timeout=0)` (non-blocking)
- Background thread uses `queue.put()` (non-blocking)

---

## Installazione Dipendenze

```bash
# Linux/Windows
pip install PyQt6 matplotlib numpy scipy pyserial

# Verifica
python -c "import PyQt6; print(PyQt6.__version__)"
python -c "import matplotlib; print(matplotlib.__version__)"
```

---

## Esecuzione

```bash
# Run ground station
python main.py
```

**Output atteso:**
```
==================================================
SimpleThermometer - Ground Station
==================================================

[SerialReader] Attempting to connect to COM3...
[SerialReader] PySerial not installed, using simulated data
[SerialReader] Thread started
[App] Closing application...
[SerialReader] Stop signal sent
[SerialReader] Thread stopped

[App] Exiting...
```

---

## Competenze Demonstrate

1. **Object-Oriented Python** — Classes, inheritance, methods
2. **Threading** — Background threads, queues, synchronization
3. **GUI Design** — PyQt6 layout, widgets, event handling
4. **Data Visualization** — matplotlib integration with PyQt
5. **Real-time Systems** — Update rates, timing, responsiveness
6. **Error Handling** — Exception handling, graceful degradation

---

## Prossimo: Fase 4

Unit Testing:
- Test framework C
- Sensor tests
- Serial parser tests

---
