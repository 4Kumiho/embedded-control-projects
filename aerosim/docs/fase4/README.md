# Fase 4 — SW OOP (Python) e Ground Station

## Cosa abbiamo fatto

Nella Fase 4 abbiamo implementato la **Ground Control Station** in Python, coprendo:
- **SW OOP (Python)**: classi, dataclasses, separazione responsabilità
- **Protocolli**: parsing di stream binari, validazione checksum
- **GUI**: interfaccia grafica real-time con PyQt6 + matplotlib

---

## File creati

| File | Ruolo |
|------|-------|
| `ground_station/parser.py` | Parser stream binario + reader file incrementale |
| `ground_station/main.py` | GUI: Artificial Horizon + Grafici + GPS Map |
| `ground_station/requirements.txt` | Dipendenze Python |

---

## Come avviare la Ground Station

```bash
# 1. Prima genera i dati con il firmware simulator
cd build && ./firmware/firmware_sim

# 2. Installa dipendenze Python
cd ../ground_station
pip install -r requirements.txt

# 3. Avvia la GCS
python main.py
```

---

## Concetto 1: Parser di protocollo binario

Il parser deve ricostruire pacchetti da uno stream di byte in arrivo.
Il problema: i byte arrivano in chunk di dimensione arbitraria — bisogna
gestire pacchetti spezzati su più letture.

### Macchina a stati implicita

```
                ┌─────────────────────────────┐
                ▼                             │
         CERCA START(0xAE)                    │
                │                             │
                ▼                             │
        LEGGI HEADER (4 byte)                 │
                │                             │
                ▼                             │
        LEGGI PAYLOAD (LENGTH byte)           │
                │                             │
                ▼                             │
        VERIFICA CHECKSUM ──── invalido ──────┘
                │ valido
                ▼
        EMETTI PACCHETTO
```

### Perché il buffer interno?

```python
def feed(self, data: bytes) -> None:
    self._buf.extend(data)   # accumula senza perdere byte

def parse_all(self) -> List[Packet]:
    # estrae tutti i pacchetti completi, lascia i byte incompleti nel buffer
```

Il buffer gestisce automaticamente i casi:
- Pacchetto arrivato in 2 read separate (spezzato)
- Più pacchetti in una sola read (accumulati)
- Byte di rumore prima del byte di start

### Deserializzazione con `struct`

```python
import struct

# Legge 6 float little-endian da 24 byte
ax, ay, az, gx, gy, gz = struct.unpack('<6f', payload)

# Legge 3 double + 1 float
lat, lon, alt = struct.unpack_from('<3d', payload, 0)   # offset 0
speed,        = struct.unpack_from('<f',  payload, 24)  # offset 24
```

Formato string: `<` = little-endian, `f` = float 4B, `d` = double 8B, `H` = uint16, `I` = uint32.

---

## Concetto 2: Lettura file incrementale (streaming)

Il file `telemetry.bin` viene scritto dal firmware in real-time.
La GCS deve leggere solo i **nuovi byte** aggiunti dall'ultima lettura:

```python
def read_new_packets(self) -> List[Packet]:
    with open(self._path, 'rb') as f:
        f.seek(self._pos)         # vai all'ultimo byte letto
        new_data = f.read()       # leggi il resto
        self._pos = f.tell()      # salva la nuova posizione
    ...
```

Questo è lo stesso principio del comando `tail -f` su Linux e di come i log server (ELK Stack) leggono file di log.

---

## Concetto 3: PyQt6 — event-driven programming

PyQt6 usa un modello **event-driven**: il codice non gira in un loop sequenziale, ma risponde a eventi.

```
Main thread (event loop)
    │
    ├── QTimer.timeout → _update() ogni 100ms
    │       └── legge nuovi pacchetti
    │           aggiorna widget
    │
    ├── Mouse click → slot collegato
    ├── Resize → paintEvent
    └── ...
```

### Signal & Slot

PyQt6 usa il pattern **Observer** tramite signal/slot:

```python
self._timer = QTimer()
self._timer.timeout.connect(self._update)  # connette signal a slot
self._timer.start(100)                     # emette signal ogni 100ms
```

Quando il timer scatta, `_update()` viene chiamato automaticamente dall'event loop.

---

## Concetto 4: QPainter — disegno vettoriale

L'**Artificial Horizon** è disegnato interamente con QPainter, senza immagini:

```python
def paintEvent(self, event):
    painter = QPainter(self)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)

    # Ruota tutto di -roll gradi attorno al centro
    painter.translate(cx, cy)
    painter.rotate(-self.roll)

    # Disegna cielo (blu) e terra (marrone) con offset pitch
    pitch_offset = self.pitch * 2.0   # pixel per grado
    painter.drawRect(...)             # cielo
    painter.drawRect(...)             # terra
```

Tecnica usata nei sistemi avionici reali (G1000, Garmin), dove i display sono generati interamente via rendering software.

### Stima assetto dall'accelerometro

```python
# Da ax, ay, az ricaviamo roll e pitch statici
roll  = atan2(ay, az)
pitch = atan2(-ax, sqrt(ay² + az²))
```

Valido solo a riposo (senza accelerazioni dinamiche). Un sistema reale usa un filtro complementare o un filtro di Kalman che fonde IMU + GPS.

---

## Concetto 5: matplotlib embedded in Qt

```python
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure

class TelemetryGraph(FigureCanvasQTAgg):
    def __init__(self):
        fig = Figure()
        super().__init__(fig)          # il canvas è anche un QWidget
        self._ax = fig.add_subplot()
        self._line, = self._ax.plot([], [])

    def refresh(self):
        self._line.set_data(x_data, y_data)
        self._ax.relim()               # ricalcola limiti
        self._ax.autoscale_view()      # aggiorna scala automaticamente
        self.draw()                    # ridisegna
```

`FigureCanvasQTAgg` è sia un widget Qt sia una figura matplotlib — può essere inserita nel layout come qualsiasi altro widget.

---

## Concetto 6: Conversione GPS → coordinate metriche

Per disegnare la mappa in metri invece di gradi:

```python
# Approssimazione piana (valida per distanze < 10 km)
north = (lat - lat0) * 111_320.0                      # m/° costante
east  = (lon - lon0) * 111_320.0 * cos(radians(lat))  # corregge per latitudine
```

Il fattore `cos(lat)` serve perché i meridiani convergono verso i poli:
a 45°N (Milano), 1° di longitudine = 78.6 km invece di 111.3 km.

---

## Struttura OOP

```
TelemetryFileReader        — legge e buffura dati dal file
    └── TelemetryParser    — macchina a stati per il parsing

ArtificialHorizon(QWidget) — widget custom con paintEvent
TelemetryGraph(FigureCanvas) — grafici real-time matplotlib
GPSMap(FigureCanvas)       — mappa traiettoria GPS

MainWindow(QMainWindow)    — finestra principale
    ├── ArtificialHorizon
    ├── TelemetryGraph
    ├── GPSMap
    └── QTimer → _update() ogni 100ms
```

---

## Prossimo passo: Fase 5

**Modello PID in Octave/Python**:
- Modello matematico del controllore PID
- Simulazione risposta a gradino
- Grafici overshoot, settling time, steady-state error
- Esportazione guadagni come `#define` per il firmware C
