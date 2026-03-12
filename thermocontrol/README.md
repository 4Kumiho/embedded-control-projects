# ThermoControl — UAV Temperature Control System

**Progetto didattico ALTEN Italia per acquisizione competenze Aerospace & Defence**

```
Temperature Sensor → PID Controller → Heater/Cooler → Simulation → GUI Display
```

---

## Quick Start

### 1. Build
```bash
cd thermocontrol
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

### 2. Run
```bash
./bin/thermocontrol_sim
```

### 3. Tests
```bash
ctest -V
pytest tests/test_ground/ -v
```

---

## Phases

| Fase | Tema | File |
|------|------|------|
| **1** | Requirements & Build System | `docs/fase1/README.md` |
| **2** | Firmware C (Sensors, PID, Telemetry) | `docs/fase2/README.md` |
| **3** | Physics Engine C++ | `docs/fase3/README.md` |
| **4** | Ground Station Python GUI | `docs/fase4/README.md` |
| **5** | Model Based Design (PID Tuning) | `docs/fase5/README.md` |
| **6** | Tests + HW Design | `docs/fase6/README.md` |

---

## Architecture

```
┌─────────────────────────────────────┐
│      ThermoControl System           │
├─────────────────────────────────────┤
│  Firmware (C)                       │
│  • Read temperature sensor (100ms)  │
│  • PID control loop (10 Hz)         │
│  • Send telemetry packets           │
├─────────────────────────────────────┤
│  Physics Simulator (C++)            │
│  • Thermal room model (dT/dt)       │
│  • Gaussian noise                   │
│  • Heater/cooler response           │
├─────────────────────────────────────┤
│  Ground Station (Python PyQt6)      │
│  • Real-time temperature plot       │
│  • Setpoint control                 │
│  • Live monitoring                  │
└─────────────────────────────────────┘
```

---

## Skills Covered (ALTEN Requirements)

- ✅ **Requirements Engineering** — SRS con requisiti formali
- ✅ **SW Embedded (C)** — Firmware con sensori, PID, scheduler
- ✅ **FW Design** — Protocol buffers, timing, interrupts
- ✅ **SW OOP (C++/Python)** — Physics engine + GUI
- ✅ **HW Design** — Schematico sensore + BOM
- ✅ **V&V** — Unit tests con coverage analysis
- ✅ **Model Based Design** — PID tuning in Octave/Python

---

## Documentation

- [SRS.md](docs/SRS.md) — System requirements
- [CLAUDE.md](CLAUDE.md) — Development guide
- [docs/fase*/README.md](docs/) — Phase-by-phase explanation

---

## Tools Required

- GCC/Clang (C/C++ compiler)
- CMake 3.15+
- Python 3.10+
- PyQt6
- GNU Octave (optional, for PID design)
- pytest (Python testing)

---

## License

Educational — ALTEN Italia Training Project
