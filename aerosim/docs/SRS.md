# Software Requirements Specification (SRS)
## AeroSim — UAV Flight Control Simulator

**Documento:** SRS-AEROSIM-001
**Versione:** 1.0
**Data:** 2026-03-10
**Autore:** Andrea
**Stato:** Draft

---

## 1. Introduzione

### 1.1 Scopo del documento
Questo documento definisce i requisiti funzionali e non funzionali del sistema **AeroSim**, un simulatore software di sistema di controllo e telemetria per UAV (Unmanned Aerial Vehicle). Il sistema è sviluppato a scopo didattico per coprire le aree tipiche dell'ingegneria embedded in ambito Aerospace & Defence.

### 1.2 Scopo del sistema
AeroSim simula il comportamento di un UAV a quadricottero in ambiente software, includendo:
- Firmware embedded (schedulazione task, gestione sensori, telemetria)
- Motore fisico per la simulazione del volo
- Ground station per monitoraggio e controllo
- Modello di controllo PID per la stabilizzazione dell'assetto

### 1.3 Definizioni e acronimi

| Termine | Definizione |
|---------|-------------|
| UAV | Unmanned Aerial Vehicle — velivolo senza pilota |
| IMU | Inertial Measurement Unit — accelerometro + giroscopio |
| GPS | Global Positioning System |
| PID | Proportional-Integral-Derivative controller |
| GCS | Ground Control Station |
| DOF | Degrees Of Freedom — gradi di libertà |
| SRS | Software Requirements Specification |
| MCU | Microcontroller Unit |
| RTOS | Real-Time Operating System |
| telemetria | trasmissione dati dal velivolo alla stazione a terra |

### 1.4 Riferimenti

| ID | Documento |
|----|-----------|
| REF-01 | STM32F4 Reference Manual (architettura MCU di riferimento) |
| REF-02 | MAVLink Protocol Specification v2.0 |
| REF-03 | DO-178C — Software Considerations in Airborne Systems |
| REF-04 | Newton-Euler equations for rigid body dynamics |

---

## 2. Descrizione generale del sistema

### 2.1 Panoramica

```
┌────────────────────────────────────────────────────────────┐
│                        AeroSim                             │
│                                                            │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐ │
│  │  Firmware   │────▶│   Physics   │────▶│   Ground    │ │
│  │  Module     │◀────│   Engine    │     │   Station   │ │
│  │  (C)        │     │   (C++)     │     │   (Python)  │ │
│  └─────────────┘     └─────────────┘     └─────────────┘ │
│         │                  │                              │
│  ┌──────▼──────────────────▼────────────────────────┐    │
│  │              PID Controller Model                 │    │
│  │              (Octave / Python)                    │    │
│  └───────────────────────────────────────────────────┘    │
└────────────────────────────────────────────────────────────┘
```

### 2.2 Funzioni principali del sistema
- Simulazione sensori (IMU, GPS, barometro) con rumore gaussiano
- Schedulazione ciclica dei task firmware (simula timer interrupt MCU)
- Modello fisico 6DOF del drone (forze e momenti)
- Controllo PID di rollio, beccheggio, imbardata e quota
- Trasmissione telemetria via protocollo binario custom
- Visualizzazione real-time su Ground Station
- Registrazione dati su file di log

### 2.3 Vincoli di progetto
- Tutto il sistema gira su PC (Windows/Linux)
- Nessun hardware fisico richiesto
- Linguaggi: C (firmware), C++ (physics), Python (GCS), Octave (MBD)
- Build system: CMake
- Copertura test: minimo 80% del codice C e C++

---

## 3. Requisiti Funzionali

### 3.1 Modulo Firmware (C)

| ID | Requisito | Priorità |
|----|-----------|----------|
| FW-001 | Il firmware deve eseguire uno scheduler ciclico con periodo configurabile (default: 10ms) | Alta |
| FW-002 | Il firmware deve simulare la lettura di un sensore IMU (accelerazione 3D + velocità angolare 3D) | Alta |
| FW-003 | Il firmware deve simulare la lettura di un sensore GPS (latitudine, longitudine, altitudine, velocità) | Alta |
| FW-004 | Il firmware deve simulare la lettura di un barometro (pressione, quota barometrica) | Media |
| FW-005 | I dati simulati dai sensori devono includere rumore gaussiano configurabile | Media |
| FW-006 | Il firmware deve costruire pacchetti di telemetria con struttura definita (header, payload, checksum) | Alta |
| FW-007 | Il firmware deve trasmettere pacchetti telemetria ogni 100ms via socket TCP | Alta |
| FW-008 | Il firmware deve ricevere comandi dalla GCS (arm/disarm, setpoint di quota e posizione) | Media |
| FW-009 | Il firmware deve eseguire un watchdog simulato: se nessun tick avviene entro 500ms, segnalare errore | Bassa |

### 3.2 Modulo Physics Engine (C++)

| ID | Requisito | Priorità |
|----|-----------|----------|
| PH-001 | Il motore fisico deve modellare il drone come corpo rigido con 6 DOF | Alta |
| PH-002 | Il motore fisico deve calcolare forze e momenti generati dai 4 rotori | Alta |
| PH-003 | Il motore fisico deve integrare le equazioni del moto con metodo Runge-Kutta 4° ordine | Alta |
| PH-004 | Il motore fisico deve simulare la forza di gravità e resistenza aerodinamica semplificata | Alta |
| PH-005 | Il motore fisico deve simulare raffiche di vento casuali | Media |
| PH-006 | Il motore fisico deve aggiornare lo stato del drone ad ogni step simulazione (default: 1ms) | Alta |
| PH-007 | Il motore fisico deve esporre interfaccia C-compatible per integrazione con il firmware | Alta |
| PH-008 | Il motore fisico deve rilevare la condizione di crash (quota < 0 o angolo > 90°) | Media |

### 3.3 Modulo Ground Station (Python)

| ID | Requisito | Priorità |
|----|-----------|----------|
| GS-001 | La GCS deve connettersi al firmware via socket TCP | Alta |
| GS-002 | La GCS deve visualizzare in real-time: rollio, beccheggio, imbardata (indicatore artificiale) | Alta |
| GS-003 | La GCS deve visualizzare la posizione GPS su mappa semplificata (griglia 2D) | Alta |
| GS-004 | La GCS deve visualizzare grafici temporali di: quota, velocità, accelerazione | Alta |
| GS-005 | La GCS deve permettere di inviare comandi di setpoint al firmware | Media |
| GS-006 | La GCS deve salvare i dati di telemetria su file CSV | Media |
| GS-007 | La GCS deve mostrare lo stato del sistema (arm/disarm, errori, latenza link) | Media |
| GS-008 | La GCS deve parsare e validare i pacchetti telemetria (checksum) | Alta |

### 3.4 Modulo PID Controller (Octave/Python)

| ID | Requisito | Priorità |
|----|-----------|----------|
| PID-001 | Il modello deve implementare un controllore PID per rollio, beccheggio e quota | Alta |
| PID-002 | Il modello deve simulare la risposta del sistema a gradino | Alta |
| PID-003 | Il modello deve produrre grafici di risposta (overshoot, settling time) | Alta |
| PID-004 | Il modello deve esportare i guadagni PID ottimizzati come header C | Media |

### 3.5 Hardware Design (KiCad)

| ID | Requisito | Priorità |
|----|-----------|----------|
| HW-001 | Lo schema deve includere un MCU (STM32F4 o equivalente) | Alta |
| HW-002 | Lo schema deve includere un modulo IMU (MPU-6050 o ICM-42688) via I2C | Alta |
| HW-003 | Lo schema deve includere un modulo GPS (u-blox NEO-M9N) via UART | Alta |
| HW-004 | Lo schema deve includere un modulo barometro (BMP390) via SPI | Media |
| HW-005 | Lo schema deve includere un modulo di comunicazione (ESP32 o nRF24) per telemetria | Media |
| HW-006 | Il PCB deve rispettare le regole DRC di KiCad senza errori | Alta |

---

## 4. Requisiti Non Funzionali

| ID | Requisito | Categoria |
|----|-----------|-----------|
| NF-001 | Il firmware deve essere compilabile con GCC senza warning (-Wall -Wextra) | Qualità |
| NF-002 | La copertura dei test (C e C++) deve essere ≥ 80% | Qualità |
| NF-003 | Il codice deve rispettare MISRA-C guidelines per le funzioni safety-critical | Standard |
| NF-004 | La latenza tra aggiornamento fisica e visualizzazione GCS deve essere < 200ms | Performance |
| NF-005 | Il sistema deve girare su Windows 10+ e Ubuntu 22.04+ | Portabilità |
| NF-006 | Ogni modulo deve avere documentazione Doxygen | Documentazione |
| NF-007 | Il repository deve avere struttura CMake con target separati per ogni modulo | Manutenibilità |

---

## 5. Protocollo di Telemetria

### 5.1 Struttura pacchetto

```
┌────────┬────────┬──────────┬─────────────────────┬──────────┐
│ START  │  TYPE  │  LENGTH  │       PAYLOAD        │ CHECKSUM │
│ 1 byte │ 1 byte │  2 byte  │    0-255 bytes       │  1 byte  │
└────────┴────────┴──────────┴─────────────────────┴──────────┘
```

- **START:** 0xAE (byte di sincronizzazione)
- **TYPE:** tipo di pacchetto (0x01=IMU, 0x02=GPS, 0x03=STATUS)
- **LENGTH:** lunghezza payload in byte (little-endian)
- **CHECKSUM:** XOR di tutti i byte del payload

### 5.2 Tipi di pacchetto

| TYPE | Nome | Payload |
|------|------|---------|
| 0x01 | IMU_DATA | ax, ay, az (float), gx, gy, gz (float) = 24 byte |
| 0x02 | GPS_DATA | lat, lon, alt (double), speed (float) = 28 byte |
| 0x03 | STATUS | arm_state (uint8), error_flags (uint16), uptime_ms (uint32) = 7 byte |

---

## 6. Casi d'uso principali

### UC-01: Avvio simulazione
1. L'utente avvia il firmware simulator
2. Il firmware inizializza scheduler e sensori simulati
3. Il firmware avvia il server TCP in ascolto
4. L'utente avvia la Ground Station
5. La GCS si connette al firmware
6. La simulazione parte e i dati fluiscono in real-time

### UC-02: Monitoraggio telemetria
1. Il firmware invia pacchetti ogni 100ms
2. La GCS riceve, valida e parsa i pacchetti
3. La GCS aggiorna i grafici e l'indicatore di assetto
4. I dati vengono salvati su CSV

### UC-03: Invio setpoint
1. L'utente inserisce un setpoint di quota nella GCS
2. La GCS invia il comando al firmware
3. Il firmware aggiorna il setpoint del PID
4. Il motore fisico risponde al nuovo comando dei motori
5. Il drone raggiunge la quota desiderata

---

## 7. Struttura del repository

```
aerosim/
├── docs/
│   ├── SRS.md                  ← questo documento
│   └── architecture.md         ← diagrammi architetturali
├── firmware/                   ← C puro
│   ├── sensors.c/h
│   ├── scheduler.c/h
│   └── telemetry.c/h
├── physics/                    ← C++
│   ├── Drone.cpp/h
│   └── Environment.cpp/h
├── ground_station/             ← Python
│   ├── main.py
│   ├── parser.py
│   └── plotter.py
├── model/                      ← Octave
│   └── pid_controller.m
├── hardware/                   ← KiCad
│   └── sensor_board/
├── tests/
│   ├── test_firmware/
│   └── test_ground/
└── CMakeLists.txt
```

---

## 8. Matrice di tracciabilità

| Requisito | Modulo | File | Test |
|-----------|--------|------|------|
| FW-001 | Firmware | scheduler.c | test_scheduler.c |
| FW-002 | Firmware | sensors.c | test_sensors.c |
| FW-006 | Firmware | telemetry.c | test_telemetry.c |
| PH-001 | Physics | Drone.cpp | test_drone.cpp |
| PH-003 | Physics | Drone.cpp | test_integration.cpp |
| GS-001 | GCS | main.py | test_connection.py |
| GS-008 | GCS | parser.py | test_parser.py |
| PID-001 | Model | pid_controller.m | — |
| HW-001 | Hardware | sensor_board.kicad_sch | DRC report |
