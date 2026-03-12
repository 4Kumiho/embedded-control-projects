# Embedded Control Projects

Monorepo contenente tre progetti didattici e di ricerca su sistemi di controllo embedded.

## Progetti

### 1. AeroSim
**Simulatore UAV multi-asse (6DOF) con PID controller**

- Simulazione dinamica 6 gradi di libertà
- Integratore RK4 per step temporali precisi
- PID tuning via Octave
- Status: ✅ Completato

**Tech:** C, C++, Octave

---

### 2. ThermoControl
**Sistema didattico di controllo termico per preparazione colloquio ALTEN**

Copre l'intero stack:
- **Firmware C:** kernel real-time, scheduler, sensori, telemetria binaria
- **Physics C++:** motore termodinamico con RK4 integrator
- **GUI Python:** PyQt6 + matplotlib, real-time plotting
- **Control Design:** analisi transfer function, PID design Octave/scipy

Fasi:
- [x] Fase 1: Requirements + CMake
- [x] Fase 2: Firmware (~4000 linee)
- [x] Fase 3: Physics Engine (~1500 linee)
- [x] Fase 4: Ground Station (~1500 linee)
- [x] Fase 5: Model Based Design (~800 linee)
- ⏳ Fase 6: Tests + HW Design

**Tech:** C, C++, Python, Octave, KiCad

---

### 3. SimpleThermometer
**Sensore termico semplificato**

Status: In progress

**Tech:** TBD

---

## Struttura

```
embedded-control-projects/
├── aerosim/                  # Simulatore UAV
├── thermocontrol/            # Sistema controllo termico
├── simplethermometer/        # Sensore termico
└── README.md
```

## Setup

Ogni progetto ha istruzioni locali nel proprio `README.md`.

## Competenze ALTEN coperte

- ✅ Requirements Engineering
- ✅ SW Embedded (C)
- ✅ FW Design
- ✅ SW OOP (C++)
- ✅ SW OOP (Python)
- ✅ Model Based Design
- ⏳ HW Design
- ⏳ V&V Testing

---

**Author:** @4Kumiho
**Status:** In development
