# Simple Thermometer — Software Requirements Specification

**Versione:** 1.0
**Data:** 2026-03-12
**Autore:** Andrea (ALTEN Training)

---

## 1. Panoramica

**SimpleThermometer** è un sistema minimalista di lettura temperatura con:
- Firmware C che legge sensore (simulato)
- GUI Python per visualizzazione
- Nessun controllo (solo monitoraggio)

**Scopo:** Imparare i fondamenti di:
- I/O sensore
- Conversione dati
- Comunicazione seriale
- Real-time plotting

---

## 2. Requisiti Funzionali

### RF1: Lettura Sensore
- Il firmware deve leggere una temperatura simulata ogni 500ms
- Temperatura compresa tra 15°C e 35°C
- Precisione: ±0.5°C

### RF2: Trasmissione Seriale
- Formato: `TEMP:XX.X°C\r\n` (es. `TEMP:22.3°C\r\n`)
- Baud rate: 115200
- Ogni 500ms un nuovo valore

### RF3: GUI in Python
- Visualizzare valore temperatura in tempo reale
- Grafico della temperatura degli ultimi 60 secondi
- Update ogni 500ms

### RF4: Test
- Testare conversione ADC → °C
- Testare parser seriale

---

## 3. Requisiti Non-Funzionali

### RNF1: Performance
- Lettura sensore: < 1ms
- Invio seriale: < 5ms
- GUI responsive: < 100ms latency

### RNF2: Robustezza
- Tollerare assenza connessione seriale (GUI mostra "waiting...")
- Reconnect automatico se seriale disconnesso

### RNF3: Semplicità
- Nessun controllo PID
- Nessuno scheduler complesso
- Loop principale semplice

---

## 4. Architettura di Alto Livello

```
┌─────────────────────────────────────────┐
│         FIRMWARE (C)                    │
│  ┌──────────────────────────────────┐   │
│  │ Sensor Model (simulate T)        │   │
│  │ → Read value                     │   │
│  │ → Convert to string              │   │
│  │ → Send via Serial                │   │
│  └──────────────────────────────────┘   │
└────────────────┬────────────────────────┘
                 │
              Serial (COM3)
                 │
┌────────────────▼────────────────────────┐
│         GROUND STATION (Python)         │
│  ┌──────────────────────────────────┐   │
│  │ Serial Reader (threading)        │   │
│  │ → Parse "TEMP:XX.X°C"            │   │
│  │ → Update plot in real-time       │   │
│  │ → Display value (PyQt6 label)    │   │
│  └──────────────────────────────────┘   │
└─────────────────────────────────────────┘
```

---

## 5. File e Componenti

### Firmware
- `firmware/main.c` — Loop principale
- `firmware/sensor.h/.c` — Lettura sensore temperatura
- `firmware/serial.h/.c` — Transmissione seriale

### Tests
- `tests/test_framework.h` — Framework minimo
- `tests/test_sensor.c` — Test conversione sensore
- `tests/test_serial_parser.c` — Test parser seriale

### Ground Station
- `ground_station/main.py` — GUI PyQt6
- `ground_station/serial_reader.py` — Lettura seriale in thread

### Model
- `model/sensor_model.md` — Transfer function sensore

---

## 6. Competenze ALTEN Coperte

✅ **Requirements Engineering** — SRS completo
✅ **SW Embedded (C)** — Firmware con sensor + serial I/O
✅ **SW OOP (Python)** — GUI con PyQt6 e threading
✅ **Testing** — Unit tests C
✅ **Model-Based Design** — Transfer function sensore

---

## 7. Success Criteria

- ✓ Firmware compila senza warning
- ✓ Legge sensore ogni 500ms ±50ms
- ✓ Trasmette formato corretto seriale
- ✓ GUI riceve e visualizza valori
- ✓ Plot aggiorna in tempo reale
- ✓ 5+ unit tests passano
- ✓ Codice commentato riga per riga

---
