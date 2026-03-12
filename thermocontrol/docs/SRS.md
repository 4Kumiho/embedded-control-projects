# ThermoControl — Software Requirements Specification (SRS)

**Versione:** 1.0
**Data:** 2026-03-12
**Autore:** Andrea (ALTEN Italia Training)

---

## 1. Scopo del Sistema

**ThermoControl** è un sistema didattico di controllo della temperatura ambiente. Acquisisce temperatura da un sensore simulato, la confronta con un setpoint desiderato e comanda un attuatore (riscaldatore/ventilatore) per mantenere il setpoint.

**Obiettivo didattico:** Imparare i fondamenti di:
- Firmware C embedded
- Controllo PID semplificato
- Physics simulation in C++
- GUI Python in tempo reale
- Verifica e validazione
- Model Based Design

---

## 2. Requisiti Funzionali

### RF.1 — Acquisizione Temperatura
**Descrizione:**
Il sistema legge la temperatura da un sensore simulato ogni 100 ms.

**Criteri di accettazione:**
- [ ] Frequenza di lettura: 100 ms ±5%
- [ ] Range sensore: 0–50 °C
- [ ] Risoluzione: 0.1 °C
- [ ] Rumore Gaussiano simulato: σ = 0.05 °C

### RF.2 — Controllo Setpoint
**Descrizione:**
L'operatore può impostare un setpoint da 15 °C a 40 °C tramite la Ground Station.

**Criteri di accettazione:**
- [ ] Setpoint modificabile in tempo reale
- [ ] Validazione range: 15–40 °C
- [ ] Comunicazione firmware ↔ PC via socket/pipe
- [ ] Latenza trasmissione < 50 ms

### RF.3 — Controllo PID
**Descrizione:**
Il firmware implementa un controllore PID per mantenere la temperatura al setpoint.

**Criteri di accettazione:**
- [ ] Errore steady-state < 0.2 °C dopo 60 sec
- [ ] Sovraelongazione < 2 °C
- [ ] Tempo di assestamento < 120 sec
- [ ] Guadagni Kp, Ki, Kd tunable

### RF.4 — Attuazione
**Descrizione:**
Il firmware invia comando di controllo (0–100%) al riscaldatore/ventilatore.

**Criteri di accettazione:**
- [ ] Comando analogico simulato (PWM equivalent)
- [ ] Limitazione output: saturation a ±100%
- [ ] Frequenza update: 10 Hz

### RF.5 — Telemetria
**Descrizione:**
Il firmware trasmette pacchetti dati di telemetria (temperatura, setpoint, errore, comando) alla Ground Station.

**Criteri di accettazione:**
- [ ] Formato: pacchetto binario custom [0xTH][TYPE][LEN][payload][XOR]
- [ ] Frequenza: 10 Hz
- [ ] Payload minimo: T_read (4B), T_setpoint (4B), error (4B), command (4B)

### RF.6 — Visualizzazione
**Descrizione:**
La Ground Station mostra grafici real-time della temperatura, setpoint e comando.

**Criteri di accettazione:**
- [ ] Grafico temperatura vs tempo (ultimi 60 sec)
- [ ] Linea setpoint sovrapposta
- [ ] Barra comando attuatore
- [ ] Refresh rate ≥ 10 Hz
- [ ] GUI responsiva (PyQt6)

### RF.7 — Simulazione Fisica
**Descrizione:**
Il modulo Physics simula il riscaldamento/raffreddamento della stanza.

**Criteri di accettazione:**
- [ ] Modello: dT/dt = (T_comando - T_ambiente) / τ + disturbi
- [ ] Costante tempo τ = 30 sec
- [ ] Rumore ambientale simulato
- [ ] Senza feedback loop instabile

---

## 3. Requisiti Non-Funzionali

| Categoria | Requisito |
|-----------|-----------|
| **Lingua** | C (firmware), C++ (physics), Python (GUI) |
| **Build** | CMake 3.15+ |
| **Piattaforma** | Windows 10/11, Linux, macOS |
| **Performance** | Latenza <100 ms da sensore a display |
| **Affidabilità** | Uptime > 95% in test 10 minuti |
| **Testabilità** | Coverage ≥ 80% per firmware C |
| **Documentazione** | Docstring + schemi ASCII per ogni fase |

---

## 4. Architettura di Sistema

```
┌──────────────────────────────────────────────────────────┐
│              ThermoControl System                        │
│                                                          │
│  ┌─────────────────┐    ┌──────────────┐               │
│  │  Firmware (C)   │───▶│ Physics (C++) │               │
│  │                 │    │ Simulation   │               │
│  │ • Leggi sensor  │    │              │               │
│  │ • PID control   │    │ • dT/dt      │               │
│  │ • Telemetry     │    │ • Rumore     │               │
│  └─────────────────┘    └──────────────┘               │
│         ▲                       │                       │
│         │ Setpoint              │ T_amb simulata       │
│         │                       ▼                       │
│  ┌──────────────────────────────────┐                  │
│  │  Ground Station (Python PyQt6)   │                  │
│  │  • Ricevi telemetry              │                  │
│  │  • Mostra grafici                │                  │
│  │  • Invia setpoint                │                  │
│  └──────────────────────────────────┘                  │
│                                                          │
│  Bridge C/C++: extern "C" in physics/thermocontrol_api.h│
└──────────────────────────────────────────────────────────┘
```

### Flusso dati
```
firmware → [telemetry packet] → socket/pipe → ground_station
                                                      ↓
                                          [plot + display]
                                                      ↓
                                    setpoint command ↓
firmware ← [command packet] ← socket/pipe ← ground_station
```

---

## 5. Protocollo Telemetria

### Pacchetto Telemetry
```
Byte  0:    0xTH          (magic byte)
Byte  1:    TYPE          (0x01 = telemetry, 0x02 = command)
Byte  2-3:  LEN (LE)      (lunghezza payload, little-endian)
Byte  4+:   PAYLOAD       (dati specifici)
Byte n-1:   XOR_CHECKSUM  (XOR di tutti i byte precedenti)
```

### Telemetry Payload
```
Offset  Tipo    Descrizione
0       float   T_read (temperatura sensore, °C)
4       float   T_setpoint (setpoint desiderato, °C)
8       float   error (T_read - T_setpoint)
12      float   command (uscita PID, -100..+100)
```

### Command Payload
```
Offset  Tipo    Descrizione
0       float   T_setpoint_new (nuovo setpoint, °C)
```

---

## 6. Fasi di Sviluppo

| Fase | Contenuto | Durata |
|------|-----------|--------|
| **1** | SRS + CMake + struttura repo | 30 min |
| **2** | Firmware C (sensore, PID base, telemetry) | 2 giorni |
| **3** | Physics Engine C++ (modello termico) | 1 giorno |
| **4** | Ground Station Python (PyQt6 + real-time plot) | 2 giorni |
| **5** | Model Based Design (PID tuning Octave) | 1 giorno |
| **6** | Tests + HW Design (schematico) | 1 giorno |

---

## 7. Metriche di Successo

- ✅ Build pass con 0 warning
- ✅ Test firmware coverage ≥ 80%
- ✅ Test Python coverage ≥ 75%
- ✅ Errore a regime < 0.2 °C
- ✅ Sovraelongazione < 2 °C
- ✅ GUI responsive (no lag > 100 ms)
- ✅ Documentazione completa per ogni fase

---

## 8. Ambiente di Sviluppo

```
Linguaggi:  C, C++, Python 3.10+
Compilatore: GCC (MinGW su Windows)
Build:      CMake 3.15+
Test C:     Custom test framework (simplice, senza dipendenze)
Test Py:    pytest
GUI:        PyQt6
Physics:    Integrazione numerica Euler (semplice RK4)
Model:      GNU Octave (alternativa Matlab)
Control Design: Octave Control Package
```

---

**Fine SRS v1.0**
