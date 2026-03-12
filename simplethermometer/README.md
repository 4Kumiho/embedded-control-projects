# SimpleThermometer — Educational Project

**Un progetto ancora più semplice per imparare i fondamenti dell'ingegneria software.**

Ogni linea di codice è commentata spiegando esattamente cosa fa.

---

## Panoramica

SimpleThermometer è un sistema di lettura temperatura con:

- **Firmware C** — Sensore simulato + trasmissione seriale
- **Python GUI** — Visualizzazione real-time + grafico
- **Unit Tests** — Test framework minimalista
- **Model-Based Design** — Analisi transfer function

**Tempo totale:** ~3-4 ore per leggere e comprendere tutto il codice

---

## Struttura Progetto

```
simplethermometer/
├── firmware/
│   ├── main.c              ← Loop principale (10 linee significative)
│   ├── sensor.h/.c         ← Simulazione sensore (100 linee + commenti)
│   └── serial.h/.c         ← Trasmissione seriale (50 linee + commenti)
│
├── ground_station/
│   ├── main.py             ← GUI PyQt6 (400 linee + commenti)
│   ├── serial_reader.py    ← Threading + seriale (300 linee + commenti)
│   └── requirements.txt
│
├── tests/
│   ├── test_framework.h    ← Mini test framework (100 linee + commenti)
│   ├── test_sensor.c       ← Sensor tests (150 linee + commenti)
│   └── test_serial_parser.c ← Parser tests (150 linee + commenti)
│
├── model/
│   └── sensor_model.md     ← Analisi matematica
│
└── docs/
    ├── SRS.md                      ← Requirements specification
    ├── fase1_requirements.md       ← Requirements engineering
    ├── fase2_firmware_c.md         ← Firmware & sensor I/O
    ├── fase3_ground_station.md     ← Python GUI & threading
    ├── fase4_testing.md            ← Unit testing
    └── fase5_model_based_design.md ← Transfer function analysis
```

---

## Competenze ALTEN Coperte

| Competenza | Fase | File |
|------------|------|------|
| Requirements Engineering | 1 | docs/SRS.md |
| SW Embedded (C) | 2 | firmware/*.c |
| SW OOP (Python) | 3 | ground_station/*.py |
| V&V (Testing) | 4 | tests/*.c |
| Model-Based Design | 5 | docs/fase5_*.md |

---

## Getting Started

### 1. Leggere Requirements (15 min)

```bash
cat docs/SRS.md
cat docs/fase1_requirements.md
```

### 2. Comprendere Firmware (1 hora)

```bash
# Leggere il codice (ogni riga commentata)
cat firmware/sensor.c
cat firmware/serial.c
cat firmware/main.c

# Compilare e eseguire
gcc -o firmware_main firmware/main.c firmware/sensor.c firmware/serial.c -lm
./firmware_main
```

Output atteso:
```
========================================
SimpleThermometer v1.0
Reading temperature every 500ms
========================================
TEMP:25.3°C
TEMP:24.8°C
TEMP:25.1°C
...
```

### 3. Eseguire Tests (30 min)

```bash
# Compilare sensor tests
gcc -o test_sensor tests/test_sensor.c firmware/sensor.c -lm
./test_sensor

# Compilare parser tests
gcc -o test_parser tests/test_serial_parser.c
./test_parser
```

### 4. Avviare GUI (1 hora)

```bash
# Installare dipendenze
pip install -r ground_station/requirements.txt

# Eseguire GUI
python ground_station/main.py
```

### 5. Studiare Model-Based Design (30 min)

```bash
cat docs/fase5_model_based_design.md
```

---

## Differenze da ThermoControl

| Aspetto | SimpleThermometer | ThermoControl |
|---------|-------------------|---------------|
| **Complessità** | Minima | Media |
| **Linee di codice** | ~1000 | ~4000 |
| **Controllo** | NO (solo lettura) | PID controller |
| **Scheduler** | Semplice loop | Task-based |
| **Tests** | 9 basic | 30+ comprehensive |
| **Model** | First-order sensor | PID tuning |
| **Commenti** | Ogni riga | Function-level |

---

## Filosofia Didattica

```
SimpleThermometer = Distillazione di ThermoControl

Rimuoviamo:
  ✗ PID controller (complesso)
  ✗ Task scheduler (complesso)
  ✗ Architecture patterns (complesso)
  ✗ Advanced testing (complesso)

Manteniamo:
  ✓ Sensor I/O
  ✓ Serial communication
  ✓ Python GUI + threading
  ✓ Unit testing basics
  ✓ Model-based design
  ✓ Realistic physics simulation

Aggiungiamo:
  + Commenti su OGNI riga di codice
  + Spiegazioni concise e dirette
  + Zero librerie esterne (oltre PyQt)
```

---

## Linea-per-Linea Commenti

Ogni file source ha commenti estesi:

```c
/* Include sensor.h header file */
#include "sensor.h"

/* Increment time counter (represents each 500ms reading) */
g_time_counter++;

/* Convert counter to angle in radians for sine wave */
float angle = (2.0f * 3.14159f * (float)g_time_counter) / 120.0f;

/* Calculate base temperature: 25°C + 5°C * sine wave */
float base_temperature = 25.0f + 5.0f * sin(angle);

/* Generate random number between 0 and 1 */
float random_fraction = (float)rand() / (float)RAND_MAX;

/* Scale random number to ±0.5°C noise */
float noise = (random_fraction - 0.5f) * 1.0f;

/* Define filter coefficient (0.1 means 10% new, 90% old) */
float alpha = 0.1f;

/* Low-pass filter: new = α*current + (1-α)*previous */
float filtered = alpha * noisy + (1.0f - alpha) * prev;
```

Ogni concetto spiegato, non presunto.

---

## Competenze Apprese

### Firmware (C)
- Variables, functions, loops
- Sensor simulation (physics model)
- String formatting
- First-order filtering
- Random number generation

### Python (OOP)
- Classes and methods
- Threading and queues
- Exception handling
- GUI layout and widgets
- Matplotlib integration

### GUI
- PyQt6 widgets
- Layouts (vertical, horizontal)
- Event handling and signals
- Background threads
- Real-time updates

### Testing
- Unit test design
- Assert macros
- Test framework creation
- Boundary testing
- Error handling

### Control Theory
- Transfer functions
- Step response
- First-order systems
- Bode plots (conceptually)
- Time constants

---

## Timing

| Fase | Attività | Tempo |
|------|----------|-------|
| 1 | Leggere SRS | 15 min |
| 2 | Capire firmware | 60 min |
| 3 | Leggere GUI | 60 min |
| 4 | Eseguire tests | 30 min |
| 5 | Studiare model | 30 min |
| | **TOTALE** | **3-4 ore** |

---

## Prossimi Passi

Dopo aver completato SimpleThermometer:

1. **Modifica il firmware:**
   - Cambia la frequenza di oscillazione (120 → 60 readings)
   - Cambia il range di temperatura (15-35 → 10-40)
   - Aggiungi nuove assert nei test

2. **Estendi la GUI:**
   - Aggiungi statistiche (min, max, media)
   - Salva dati in CSV
   - Connetti a veri serial ports

3. **Aggiungi controllo:**
   - Implementa semplice P controller
   - Evolvere a PI, poi PID
   - Analizza step response

4. **Approfondisci il testing:**
   - Aggiungi pytest tests in Python
   - Crea CI pipeline
   - Coverage reporting

---

## File Key

### Per Capire il Concetto Generale
- `docs/SRS.md` — What (15 min read)
- `docs/fase1_requirements.md` — Why (10 min read)

### Per Capire il Firmware
- `firmware/sensor.c` — How sensor works (30 min read)
- `firmware/serial.c` — How to transmit (10 min read)
- `firmware/main.c` — How it all ties together (5 min read)

### Per Capire la GUI
- `ground_station/serial_reader.py` — Threading + parsing (40 min read)
- `ground_station/main.py` — PyQt6 GUI (40 min read)

### Per Capire i Tests
- `tests/test_framework.h` — Framework (15 min read)
- `tests/test_sensor.c` — Sensor tests (20 min read)

### Per Capire la Teoria
- `docs/fase5_model_based_design.md` — Control theory (30 min read)

---

## Support / Questions

Ogni linea di codice ha un commento. Se una linea non è chiara:

1. Leggi il commento sopra/accanto
2. Leggi la funzione che la contiene
3. Leggi il file di documentazione (docs/fase*_*.md)

Se ancora non chiara, la documentazione ha una lacuna da migliorare.

---

## License

Educational project - ALTEN Training

---

## References

- "The C Programming Language" by Kernighan & Ritchie
- PyQt6 Documentation: https://www.riverbankcomputing.com/static/Docs/PyQt6/
- Matplotlib Documentation: https://matplotlib.org/
- "Modern Control Systems" by Dorf & Bishop

---
