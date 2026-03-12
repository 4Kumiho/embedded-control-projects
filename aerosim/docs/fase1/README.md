# Fase 1 — Requirements Engineering & Setup del Progetto

## Cosa abbiamo fatto

Nella Fase 1 abbiamo gettato le fondamenta del progetto AeroSim, coprendo la prima competenza richiesta da ALTEN: **Requirements & System Engineering**.

---

## 1. Requirements & System Engineering

Abbiamo scritto il documento **SRS (Software Requirements Specification)**, il documento fondamentale in qualsiasi progetto ingegneristico professionale.

### Cos'è un SRS
Un SRS descrive **cosa** il sistema deve fare (non come). È il contratto tra chi commissiona il sistema e chi lo sviluppa. In ambito Aerospace & Defence è obbligatorio seguire standard come DO-178C (software aviazione) o ECSS (spazio europeo).

### Struttura del nostro SRS

| Sezione | Contenuto |
|---------|-----------|
| Introduzione | Scopo, acronimi, riferimenti normativi |
| Descrizione generale | Architettura ad alto livello, funzioni principali |
| Requisiti Funzionali | Cosa il sistema DEVE fare (verificabile) |
| Requisiti Non Funzionali | Performance, qualità, portabilità, standard |
| Protocollo Telemetria | Formato binario dei pacchetti di dati |
| Casi d'uso | Scenari tipici di utilizzo del sistema |
| Matrice di tracciabilità | Collegamento requisito → file → test |

### Tipi di requisiti scritti

**Requisiti di alto livello** — descrivono il comportamento del sistema visto dall'esterno:
- Esempio: *"Il sistema deve trasmettere telemetria ogni 100ms"*

**Requisiti di basso livello** — scendono nel dettaglio implementativo:
- Esempio: *"Il pacchetto telemetria ha struttura: START(1B) + TYPE(1B) + LENGTH(2B) + PAYLOAD + CHECKSUM(1B)"*

**Requisiti non funzionali** — qualità trasversali:
- Esempio: *"Il firmware deve compilare senza warning con -Wall -Wextra"*
- Esempio: *"La copertura dei test deve essere ≥ 80%"*

### Requisiti scritti per modulo

| Modulo | Numero requisiti | ID |
|--------|-----------------|-----|
| Firmware (C) | 9 | FW-001 … FW-009 |
| Physics Engine (C++) | 8 | PH-001 … PH-008 |
| Ground Station (Python) | 8 | GS-001 … GS-008 |
| PID Controller (Octave) | 4 | PID-001 … PID-004 |
| Hardware (KiCad) | 6 | HW-001 … HW-006 |
| Non funzionali | 7 | NF-001 … NF-007 |

---

## 2. Architettura del sistema definita

Abbiamo definito l'architettura ad alto livello del sistema:

```
Firmware (C)  ──▶  Physics Engine (C++)  ──▶  Ground Station (Python)
     │                     │
     └─────────────────────┴──▶  PID Model (Octave)
```

Ogni blocco corrisponde a una competenza richiesta da ALTEN e a un linguaggio diverso.

---

## 3. Protocollo di comunicazione definito

Abbiamo progettato il **protocollo binario di telemetria** usato per comunicare tra Firmware e Ground Station:

```
┌────────┬────────┬──────────┬─────────────────────┬──────────┐
│ START  │  TYPE  │  LENGTH  │       PAYLOAD        │ CHECKSUM │
│ 1 byte │ 1 byte │  2 byte  │    0-255 bytes       │  1 byte  │
└────────┴────────┴──────────┴─────────────────────┴──────────┘
```

Questo è un esempio reale di come funzionano protocolli come MAVLink (usato nei droni reali) o SpaceWire (usato nei satelliti ESA).

---

## 4. Build system configurato

Abbiamo impostato **CMake**, il build system standard nell'industria embedded/aerospace:

```
aerosim/
├── CMakeLists.txt          ← root: C11, C++17, -Wall -Wextra
├── firmware/CMakeLists.txt ← libreria statica firmware
├── physics/CMakeLists.txt  ← libreria statica physics
└── tests/CMakeLists.txt    ← framework di test
```

Per compilare il progetto basterà:
```bash
mkdir build && cd build
cmake ..
make
```

---

## 5. Struttura del repository

```
aerosim/
├── docs/
│   ├── SRS.md              ← documento dei requisiti
│   └── fase1/
│       └── README.md       ← questo file
├── firmware/               ← Fase 2 (C)
├── physics/                ← Fase 3 (C++)
├── ground_station/         ← Fase 4 (Python)
├── model/                  ← Fase 5 (Octave)
├── hardware/               ← Fase 6 (KiCad)
├── tests/                  ← Fase 6 (V&V)
├── CMakeLists.txt
├── .gitignore
└── README.md
```

---

## Concetti chiave appresi

- **SRS** — documento fondamentale in ogni progetto safety-critical
- **Requisiti funzionali vs non funzionali** — cosa fa il sistema vs come lo fa bene
- **Matrice di tracciabilità** — ogni requisito deve essere verificabile con un test
- **CMake** — build system multi-piattaforma standard nel mondo embedded
- **Protocollo binario** — come i sistemi embedded comunicano in modo efficiente

---

## Prossimo passo: Fase 2

Sviluppo del **Firmware in C**:
- `scheduler.c` — schedulatore ciclico dei task (simula i timer interrupt dell'MCU)
- `sensors.c` — driver simulati per IMU e GPS con rumore gaussiano
- `telemetry.c` — costruzione e trasmissione pacchetti con il protocollo definito
