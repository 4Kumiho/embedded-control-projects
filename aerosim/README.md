# AeroSim — UAV Flight Control Simulator

Simulatore software di sistema di controllo e telemetria per UAV, sviluppato per coprire le aree tipiche dell'ingegneria embedded in ambito Aerospace & Defence.

## Struttura del progetto

| Modulo | Linguaggio | Competenza |
|--------|-----------|------------|
| `firmware/` | C | SW Embedded, FW |
| `physics/` | C++ | SW OOP |
| `ground_station/` | Python | SW OOP |
| `model/` | Octave | Model Based Design |
| `hardware/` | KiCad | HW Design |
| `tests/` | C / Python | V&V |
| `docs/SRS.md` | Markdown | Requirements Engineering |

## Build

```bash
mkdir build && cd build
cmake ..
make
```

## Fasi di sviluppo

- [x] Fase 1 — SRS + struttura repo
- [ ] Fase 2 — Firmware C (sensori, scheduler, telemetria)
- [ ] Fase 3 — Physics Engine C++
- [ ] Fase 4 — Ground Station Python
- [ ] Fase 5 — Modello PID Octave
- [ ] Fase 6 — HW KiCad + Test V&V
