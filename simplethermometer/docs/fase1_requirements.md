# Fase 1 — Requirements Engineering

**Durata:** ~15 minuti
**Competenze coperte:** Requirements Engineering, Specification

---

## Panoramica

Fase 1 definisce i requisiti funzionali e non-funzionali per SimpleThermometer.

Risultato: **SRS.md** con specifica completa.

---

## Competenza ALTEN Coperta

✅ **Requirements Engineering** — SRS (Software Requirements Specification)

---

## File Creati

### SRS.md — Software Requirements Specification

**Contiene:**
- Panoramica progetto
- Requisiti funzionali (RF1-RF4)
- Requisiti non-funzionali (RNF1-RNF3)
- Architettura di alto livello (diagramma)
- Lista componenti
- Success criteria

**Sezioni principali:**

```
RF1: Lettura Sensore
  - Legge temperatura ogni 500ms
  - Range: 15-35°C
  - Precisione: ±0.5°C

RF2: Trasmissione Seriale
  - Formato: "TEMP:XX.X°C\r\n"
  - Baud: 115200

RF3: GUI in Python
  - Visualizzazione real-time
  - Grafico ultimi 60 secondi
  - Update ogni 500ms

RF4: Test
  - Test conversione sensore
  - Test parser seriale
```

---

## Architettura (da SRS)

```
┌─────────────────────────┐
│ FIRMWARE (C)            │
│ - Sensor Model          │
│ - Serial TX             │
└──────────┬──────────────┘
           │ Serial (COM3)
           │
┌──────────▼──────────────┐
│ GROUND STATION (Python) │
│ - Serial Reader Thread  │
│ - PyQt6 GUI             │
│ - Matplotlib Plot       │
└─────────────────────────┘
```

---

## Competenze Demonstrate

In Fase 1:

1. **Requirements Analysis** — Definire cosa deve fare il sistema
2. **Specification Writing** — Documentare in modo chiaro e completo
3. **Architecture Design** — Disegnare il flusso dati ad alto livello
4. **Success Criteria** — Definire come verificare che il progetto è completo

---

## Prossimo: Fase 2

Implementazione Firmware C:
- `firmware/sensor.c` — Lettura sensore simulato
- `firmware/serial.c` — Trasmissione seriale
- `firmware/main.c` — Loop principale

---
