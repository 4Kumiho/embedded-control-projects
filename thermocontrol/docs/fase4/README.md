# Fase 4 — Ground Station Python (GUI & Telemetry Display)

**Durata:** ~2 giorni di studio approfondito
**Competenze coperte:** SW OOP (Python), GUI Development, Network Programming, Real-time Visualization

---

## Panoramica

Fase 4 implementa la Ground Station — l'applicazione Python che:
1. Riceve telemetria dal firmware (pacchetti binari via socket)
2. Decodifica i dati
3. Visualizza grafici real-time
4. Permette all'operatore di controllare il sistema (cambio setpoint)

Completa il ciclo: Firmware → Physics → Ground Station

---

## Architettura

```
Firmware (Fase 2)
    ↓ TCP packets [21 bytes each]
Ground Station (Fase 4)
    ├─ Network Thread (TelemetryWorker)
    │  └─ Receives & parses binary packets
    ├─ GUI Thread (PyQt6)
    │  ├─ Real-time plot (matplotlib)
    │  ├─ Status display
    │  ├─ Control slider
    │  └─ Statistics
    └─ User interaction
       └─ Slider → command packet → firmware
```

---

## File creati

### 1. parser.py — Binary Protocol Parser

**Cosa fa:**
- Decodifica pacchetti binari telemetria (21 byte)
- Encoda comandi per inviare al firmware
- Verificazione checksum XOR

**Funzioni principali:**

```python
# Parsing
packet = parse_telemetry_frame(frame_bytes)  # bytes → TelemetryPacket
packets = parse_stream(data)                 # Multi-frame parsing

# Encoding
frame = encode_command(25.0)                 # float → command packet

# Verification
verify_checksum(frame)                       # Check XOR integrity
format_packet(packet)                        # Pretty print

# Testing
pytest parser.py                             # Unit tests
```

**Struttura TelemetryPacket:**

```python
@dataclass
class TelemetryPacket:
    temperature: float   # °C
    setpoint: float      # °C
    error: float         # °C
    command: float       # %
    timestamp: float     # seconds
```

**Protocollo binario (21 byte):**

```
Byte 0:    0x54        Magic byte
Byte 1:    0x01        Type = telemetry
Byte 2-3:  0x10 0x00   Payload length (16 bytes)
Byte 4-7:  [float]     Temperature
Byte 8-11: [float]     Setpoint
Byte 12-15: [float]    Error
Byte 16-19: [float]    Command
Byte 20:   [XOR]       Checksum
```

**XOR Checksum:**
```python
checksum = 0
for byte in frame[:-1]:
    checksum ^= byte
frame[-1] == checksum  # Verify
```

**Competenza ALTEN:** SW OOP (Python), Binary protocols, Unit testing

---

### 2. requirements.txt — Python Dependencies

**Dipendenze:**
- **PyQt6** — GUI framework (Qt for Python)
- **numpy** — Numerical computing
- **matplotlib** — Plotting library
- **scipy** — Scientific computing (optional)

**Install:**
```bash
pip install -r requirements.txt
```

---

### 3. main.py — PyQt6 Ground Station Application

**Cosa fa:**
- Crea interfaccia PyQt6 con plotting real-time
- Gestisce connessione TCP al firmware
- Riceve pacchetti telemetria
- Visualizza temperatura/setpoint/comando
- Permette all'operatore di inviare comandi

**Architettura:**

```
┌─────────────────────────────────────────┐
│ ThermoControlWindow (QMainWindow)        │
├─────────────────────────────────────────┤
│ Status Label (current T, Tset, err, cmd)│
├─────────────────────────────────────────┤
│                                         │
│   matplotlib Figure with Axes           │
│   ├─ Primary: Temperature (blue line)  │
│   ├─         Setpoint (red dashed)     │
│   └─ Secondary: Command (green bars)   │
│                                         │
│   Auto-scroll: shows last 60 seconds    │
│                                         │
├─────────────────────────────────────────┤
│ Setpoint Slider: 15-40°C                │
│ [Send Command] [Reset Plot] [Auto...]   │
│ Statistics: packets, avg rate, temp range│
└─────────────────────────────────────────┘
```

**Componenti chiave:**

#### TelemetryWorker (QObject in QThread)
```python
class TelemetryWorker(QObject):
    packet_received = pyqtSignal(TelemetryPacket)
    connection_status_changed = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)

    def run(self):
        # Runs in separate thread
        # Connects to firmware
        # Receives packets in loop
        # Emits signals (thread-safe)
```

**Perché thread separato?**
- TCP socket recv() è bloccante
- Se in main GUI thread, UI si freezerebbe
- Worker thread gestisce I/O, main thread gestisce GUI
- Segnali PyQt garantiscono thread-safety

#### Circular Buffers
```python
self.time_data = deque(maxlen=600)       # Last 600 seconds
self.temp_data = deque(maxlen=600)       # Circular buffer
self.setpoint_data = deque(maxlen=600)
self.command_data = deque(maxlen=600)
```

**Vantaggi:**
- Memory bounded (sempre max 600 elementi)
- FIFO: nuovi dati sovrescrivono vecchi
- Efficienti: O(1) append/pop

#### Real-Time Plotting
```python
def update_plot(self):
    # Called every 33ms (~30 Hz refresh rate)
    # Draws fresh plot from buffer data
    # matplotlib canvas.draw_idle() updates display
```

**Subplot dual-axis:**
```python
ax1 = figure.add_subplot(111)
ax1.plot(times, temps, 'b-')             # Temperature (blue, left axis)
ax1.plot(times, setpoints, 'r--')        # Setpoint (red, left axis)

ax2 = ax1.twinx()                         # Create second y-axis
ax2.bar(times, commands, color='g')      # Command (green, right axis)
```

Questo permette di visualizzare:
- Temperatura e setpoint in °C (asse sinistro)
- Comando in % (asse destro)
Su stessa finestra senza conflitto di scala.

#### Control Panel
```python
# Setpoint slider (15-40°C)
slider = QSlider(Qt.Horizontal)
slider.setMinimum(15)
slider.setMaximum(40)
slider.sliderMoved.connect(self.on_setpoint_changed)

# When user moves slider:
# 1. on_setpoint_changed() is called
# 2. Encode command with new setpoint
# 3. Send via socket to firmware
# 4. Firmware receives, updates PID setpoint
# 5. Next telemetry packet shows new setpoint
```

**Competenza ALTEN:** GUI Development (PyQt6), Real-time visualization, Threading

---

## Flusso dati Fase 2 ↔ Fase 4

```
main.c (Firmware, Fase 2)
    ├─ TASK_TELEMETRY
    │  └─ telemetry_encode(T, Tset, err, cmd)
    │     → 21-byte packet
    │     → Send via socket
    ↓
Ground Station (Fase 4, main.py)
    ├─ TelemetryWorker.run()
    │  └─ socket.recv(4096)
    │     → Accumula in buffer
    │     → Cerca magic byte (0x54)
    │     → Estrae frame 21 byte
    │     → parse_telemetry_frame()
    │     → TelemetryPacket
    │     → Emette signal: packet_received()
    ↓
MainWindow.on_packet_received()
    ├─ Aggiunge a buffers circolari
    ├─ Aggiorna status label
    ↓
update_plot() (timer ogni 33ms)
    ├─ Legge buffers
    ├─ Disegna matplotlib figura
    ├─ Show ultimi 60 secondi (auto-scroll)
    ↓
User vede grafico real-time!

---

User muove slider setpoint:
    ↓
on_setpoint_changed()
    ├─ Legge valore slider
    ├─ encode_command(setpoint)
    ├─ worker.send_command(setpoint)
    ├─ socket.sendall(9-byte packet)
    ↓
Firmware riceve comando
    └─ pid_set_setpoint(new_setpoint)
       → PID controller aggiorna target
       → Temperature gradualmente raggiunge nuovo setpoint
```

---

## Threading Model

PyQt6 + Threading:

```
Main Thread (GUI)
├─ QApplication event loop
├─ User events (slider, button clicks)
├─ Timer callbacks (update_plot)
└─ Signal slots (on_packet_received)

Worker Thread
├─ TelemetryWorker.run()
├─ socket.recv() (blocking!)
├─ parse packet
└─ emit signal (thread-safe)

Communication:
    Worker emits: packet_received.emit(packet)
    ↓
    Main thread slot: on_packet_received(packet)

    Signals are thread-safe thanks to Qt's metacall system
```

**Perché non socket.setblocking(False)?**
- Non-blocking socket richiederebbe poll loop
- Più complesso da gestire
- Blocking + thread è semplice e chiaro
- Modern approach: uno thread per I/O, uno per UI

---

## Test & Debug

### Unit Test parser.py
```bash
cd ground_station
python -m pytest parser.py -v

# Output:
test_xor_checksum PASSED
test_checksum_verify PASSED
test_parse_telemetry PASSED
test_parse_invalid_magic PASSED
test_encode_command PASSED
```

### Run Ground Station
```bash
python main.py
```

Expected behavior:
1. Window opens
2. Tries to connect to firmware at 127.0.0.1:5555
3. If firmware running (thermocontrol_sim), connection succeeds
4. Telemetry data starts arriving
5. Plot updates every 33ms with new data
6. User can move slider to change setpoint
7. Firmware responds, temperature changes
8. Plot shows change in real-time

---

## GUI Features

### Status Display
```
T=21.35°C | Tset=25.00°C | err=3.65°C | cmd=27.3% | packets=125
```
Updates on every incoming packet (10 Hz)

### Real-Time Plot
- **X-axis:** Time (seconds, last 60 when auto-scroll on)
- **Y-axis (left):** Temperature °C (15-40 range)
- **Y-axis (right):** Command % (-100 to +100)
- **Blue line:** Current temperature (noisy)
- **Red dashed:** Setpoint (smooth target)
- **Green bars:** Heater command output

### Control Panel
- **Setpoint slider:** 15-40°C continuous
- **Send Command button:** Manually send slider value
- **Reset Plot button:** Clear all data, start fresh
- **Auto-scroll checkbox:** Toggle 60-sec window vs full history
- **Statistics box:** Packet count, avg rate, temp range

---

## Competenze ALTEN Coperte

| Competenza | Dove | Come |
|-----------|------|------|
| **SW OOP (Python)** | parser.py, main.py | Classes, dataclasses, signals |
| **GUI Development** | PyQt6, matplotlib | Layouts, widgets, event loop |
| **Network Programming** | TelemetryWorker | Sockets, TCP, stream parsing |
| **Real-Time Systems** | Threading, signals | Responsive UI, responsive network |
| **Data Visualization** | matplotlib | Dual-axis plots, real-time update |
| **Binary Protocols** | parser.py | Encoding/decoding, checksums |

---

## Architettura Software

```
Ground Station Python
    ├─ GUI Layer (PyQt6)
    │  ├─ MainWindow (QMainWindow)
    │  ├─ Controls (slider, buttons)
    │  ├─ Plot (matplotlib canvas)
    │  └─ Status display
    │
    ├─ Threading Layer
    │  ├─ MainWindow (main thread, ~GUI event loop)
    │  └─ TelemetryWorker (worker thread, ~network I/O)
    │
    ├─ Network Layer
    │  ├─ TCP socket (blocking)
    │  └─ Stream buffer + frame extraction
    │
    └─ Protocol Layer
       ├─ parser.py (encode/decode)
       ├─ XOR checksum (error detection)
       └─ Binary frame [0x54][TYPE][LEN][...][XOR]
```

---

## Performance

- **Plot update:** 30 Hz (every 33ms)
- **Telemetry receive:** 10 Hz (100ms period from firmware)
- **Memory:** ~5MB for circular buffers + matplotlib
- **CPU:** <5% idle, ~10% during plot update (depends on OS)

---

## Possibili miglioramenti (per futuro)

1. **Data logging:** Salva CSV con timestamp
2. **Playback:** Carica file CSV precedente e replay
3. **Statistics:** Mean, stddev, min/max di T
4. **PID tuning widget:** Interactive Kp/Ki/Kd sliders
5. **Network settings:** GUI per host/port configuration
6. **Alarm thresholds:** Alert if T > T_max
7. **Export plots:** Save matplotlib figure as PNG
8. **Multi-axis:** Show derivative (dT/dt) on third axis

---

## Prossime fasi

- **Fase 5:** Model Based Design (Octave PID tuning)
- **Fase 6:** Tests & Hardware Design (CTest, pytest, KiCad)

---

## Referenze

### PyQt6
- https://www.riverbankcomputing.com/software/pyqt/
- https://doc.qt.io/qt-6/ (Qt documentation)

### matplotlib
- https://matplotlib.org/
- Real-time plotting: https://matplotlib.org/stable/users/animation.html

### Threading in PyQt
- https://doc.qt.io/qt-6/qthread.html
- "Threading without using QThread" anti-pattern

### Socket Programming (Python)
- https://docs.python.org/3/library/socket.html
- "Python Sockets Tutorial" for details
