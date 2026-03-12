# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Progetto

**AeroSim** — simulatore software di controllo e telemetria per UAV (quadricottero), sviluppato per coprire le aree dell'ingegneria embedded in ambito Aerospace & Defence richieste da ALTEN Italia. Tutto gira su PC, nessun hardware fisico necessario.

## Build (C/C++)

```bash
# Prima build
mkdir build && cd build
cmake ..
make

# Target specifici
make firmware_sim     # eseguibile firmware
make test_firmware    # suite test C

# Eseguire il firmware simulator (genera telemetry.bin nella cartella build/)
cd build && ./firmware/firmware_sim

# Eseguire tutti i test C con CTest
cd build && ctest -V

# Eseguire solo i test firmware
cd build && ./tests/test_firmware/test_firmware
```

## Test Python

```bash
# Dalla root del progetto
cd tests/test_ground
pytest test_parser.py -v

# Singolo test
pytest test_parser.py::TestStreamHandling::test_partial_then_complete -v

# Tutti i test Python
pytest tests/test_ground/ -v --tb=short
```

## Ground Station

```bash
cd ground_station
pip install -r requirements.txt

# La GCS legge build/telemetry.bin generato da firmware_sim
# Avviare firmware_sim prima, poi:
python main.py
```

## Model Based Design

```bash
# Octave
cd model && octave pid_controller.m

# Python (alternativa, stessa analisi)
cd model && python pid_analysis.py
# Genera: model/pid_analysis.png e riscrive model/pid_gains.h
```

## Architettura

Il sistema ha 5 componenti che comunicano attraverso due interfacce:

```
firmware/ (C)  ──► telemetry.bin ──► ground_station/ (Python)
     │
     │  drone_c_api.h (extern "C")
     ▼
physics/ (C++)

model/ (Octave/Python) ──► model/pid_gains.h ──► firmware/ (C)
```

### Flusso dati runtime

1. `firmware_sim` gira il loop scheduler → campiona sensori simulati → scrive pacchetti binari in `build/telemetry.bin`
2. `ground_station/main.py` legge `telemetry.bin` in modo incrementale (seek + read) ogni 100ms → parsea i pacchetti → aggiorna GUI

### Protocollo telemetria binario

Tutti i pacchetti: `[0xAE][TYPE][LEN_LO][LEN_HI][...payload...][XOR_checksum]`

| TYPE | Payload | Dimensione totale |
|------|---------|-------------------|
| 0x01 IMU | 6× float LE (ax,ay,az,gx,gy,gz) | 29 byte |
| 0x02 GPS | 3× double LE + 1× float LE | 33 byte |
| 0x03 STATUS | uint8 + uint16 LE + uint32 LE | 12 byte |

Il parser Python (`ground_station/parser.py`) e il parser C nei test (`tests/test_firmware/test_telemetry.c`) devono restare sincronizzati con queste dimensioni.

### Interfaccia C/C++

`physics/drone_c_api.h` è l'unico header che il firmware C può includere per accedere al physics engine C++. Usa `extern "C"` per evitare il name mangling. Le strutture `CDroneState` e `CMotorCommands` sono C-compatible (no classi, no riferimenti).

### Scheduler firmware

Lo scheduler (`firmware/scheduler.c`) è cooperativo: `scheduler_tick()` va chiamato nel main loop. La sottrazione unsigned `(now - last_run_ms)` gestisce il wrap-around uint32 senza if speciali. Max 8 task (`SCHEDULER_MAX_TASKS`).

### Modello fisico

`physics/Drone.cpp` integra le equazioni di Newton-Euler con RK4 (passo default 1ms). `compute_derivative()` è il cuore: viene chiamata 4 volte per step RK4. La matrice di rotazione usa convenzione ZYX (yaw → pitch → roll). Crash detection: quota < 0 oppure |roll| o |pitch| > 75°.

### PID gains

`model/pid_gains.h` è generato automaticamente da `model/pid_controller.m` o `model/pid_analysis.py`. Non modificarlo a mano — rieseguire il modello e lasciare che venga sovrascritto.

## Standard di codice

- **C**: C11, `-Wall -Wextra -Wpedantic`, commenti Doxygen su tutte le funzioni pubbliche, funzioni private `static`
- **C++**: C++17, stessi flag, `const`-correctness obbligatoria
- **Float**: usare `ASSERT_NEAR` nei test C (mai `==` su float); stessa logica in pytest con `abs(a - b) < tol`
- **Sensori stocastici**: i test statistici usano N=1000 campioni con seed fisso per riproducibilità (`sensors_init(42)`)
- **Serializzazione**: tutti i tipi multi-byte sono little-endian; usare `memcpy` per float/double (mai cast diretto)

## Documentazione per fase

Ogni fase ha la propria guida didattica in `docs/faseN/README.md`. La specifica di sistema completa è in `docs/SRS.md` (requisiti FW-*, PH-*, GS-*, PID-*, HW-*, NF-*).
