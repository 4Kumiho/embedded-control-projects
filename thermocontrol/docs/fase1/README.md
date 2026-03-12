# Fase 1 — Requirements Engineering & Build System

**Cosa abbiamo fatto:**

## 1. SRS.md — Software Requirements Specification

Documento formale che definisce:

- **Requisiti Funzionali (RF):**
  - RF.1: Acquisizione temperatura (sensore simulato, 100 ms)
  - RF.2: Controllo setpoint (15–40 °C da GUI)
  - RF.3: Controllo PID (errore < 0.2 °C)
  - RF.4: Attuazione (PWM simulato 0–100%)
  - RF.5: Telemetria (pacchetti binari)
  - RF.6: Visualizzazione (GUI PyQt6 real-time)
  - RF.7: Simulazione fisica (modello termico)

- **Requisiti Non-Funzionali:**
  - Linguaggi: C, C++, Python
  - Build: CMake 3.15+
  - Testabilità: coverage ≥ 80%
  - Performance: latenza < 100 ms

- **Architettura di sistema:** Flow diagram firmware ↔ physics ↔ GUI

- **Protocollo telemetria:** Pacchetto binario [0xTH][TYPE][LEN][payload][XOR]

---

## 2. CMakeLists.txt — Build System

### Root CMakeLists.txt
```cmake
project(ThermoControl VERSION 1.0.0)
cmake_minimum_required(VERSION 3.15)
add_subdirectory(firmware)
add_subdirectory(physics)
add_subdirectory(tests)
```

**Riga per riga:**

| Riga | Significato |
|------|------------|
| `cmake_minimum_required(3.15)` | CMake versione minima richiesta |
| `project(ThermoControl)` | Nome progetto + versione |
| `set(CMAKE_C_STANDARD 99)` | Standard C per firmware |
| `set(CMAKE_CXX_STANDARD 14)` | Standard C++ per physics |
| `add_compile_options(-Wall -Wextra)` | Abilita tutti gli warning |
| `add_subdirectory(firmware)` | Compila firmware/ |
| `add_subdirectory(physics)` | Compila physics/ |
| `add_subdirectory(tests)` | Compila tests/ |

### firmware/CMakeLists.txt
```cmake
add_library(thermocontrol_fw ...)  # Libreria firmware (moduli C)
add_executable(thermocontrol_sim main.c)  # Eseguibile simulatore
target_link_libraries(...)  # Collega fisica (C/C++ bridge)
```

### physics/CMakeLists.txt
```cmake
add_library(thermocontrol_physics ...)  # Libreria C++ (modello termico)
target_compile_features(cxx_std_14)  # C++14 per std::vector, lambda
```

### tests/CMakeLists.txt
```cmake
add_subdirectory(test_firmware)  # Test unitari C
add_subdirectory(test_ground)    # Test Python (via pytest)
add_test(...)  # Registra test con CTest
```

---

## 3. Struttura Directory

```
thermocontrol/
├── docs/                        # Documentazione
│   ├── SRS.md                   # Requisiti di sistema
│   ├── fase1/                   # Questa cartella
│   ├── fase2/ ... fase6/        # Fasi successive
├── firmware/                    # Codice C (microcontrollore simulato)
│   ├── main.c                   # Entry point
│   ├── sensors.c/h              # Driver sensore temperatura
│   ├── pid_controller.c/h       # Algoritmo PID
│   ├── telemetry.c/h            # Protocollo trasmissione dati
│   ├── scheduler.c/h            # Task scheduler
│   ├── platform.c/h             # Astrazione OS (timer, sleep)
│   └── CMakeLists.txt
├── physics/                     # Codice C++ (simulazione)
│   ├── Room.h/cpp               # Modello stanza (termico)
│   ├── ThermoModel.h/cpp        # Equazione differenziale dT/dt
│   ├── thermocontrol_c_api.h/cpp # Bridge C/C++
│   └── CMakeLists.txt
├── ground_station/              # Codice Python (GUI)
│   ├── main.py                  # Finestra PyQt6
│   ├── parser.py                # Parsing telemetria binaria
│   ├── plotter.py               # Grafici real-time
│   └── requirements.txt
├── model/                       # Model Based Design
│   ├── pid_controller.m         # Modello PID Octave
│   ├── pid_analysis.py          # Analisi Python (scipy)
│   └── pid_gains.h              # Header C generato
├── hardware/                    # Documentazione HW
│   └── sensor_board/
│       ├── schematic_description.md
│       └── BOM.csv
├── tests/                       # Test suite
│   ├── test_firmware/           # Test C (CTest)
│   └── test_ground/             # Test Python (pytest)
├── CMakeLists.txt               # Root build config
├── CLAUDE.md                    # Guida per Claude Code
├── README.md                    # Quick start
└── .gitignore
```

---

## 4. Key Concepts

### 4.1 Requirements Engineering
- **SRS** = documento formale con requisiti funzionali e non-funzionali
- **RF** (Requisiti Funzionali) = cosa deve fare il sistema
- **RNF** (Requisiti Non-Funzionali) = come deve farlo (performance, linguaggio, etc.)
- **Criteri di accettazione** = come verificare che il requisito è soddisfatto

### 4.2 Build System (CMake)
- **CMakeLists.txt** = ricetta di compilazione (platform-independent)
- **add_executable()** = crea un .exe o binario
- **add_library()** = crea una libreria riutilizzabile (.a/.lib/.dll)
- **target_link_libraries()** = collega librerie tra loro
- **add_subdirectory()** = compila subdirectory (modularità)

### 4.3 Architettura Modulare
```
Main
 ├─ Firmware (C)
 │   ├─ sensors.h   (lettura dato)
 │   ├─ pid_controller.h (elaborazione)
 │   ├─ telemetry.h (trasmissione)
 │   └─ scheduler.h (timing)
 │
 ├─ Physics (C++)
 │   ├─ Room (entità)
 │   ├─ ThermoModel (equazione)
 │   └─ C API (collegamento con C)
 │
 └─ GUI (Python)
     ├─ parser.py (decifra pacchetti)
     ├─ plotter.py (mostra grafici)
     └─ main.py (coordinatore)
```

### 4.4 Protocollo Binario
```
Byte 0:     [0xTH]            Magic marker
Byte 1:     [TYPE]            01=telemetry, 02=command
Byte 2-3:   [LEN_LE]          Payload length (little-endian)
Byte 4+:    [PAYLOAD]         4 float = 16 byte
Byte n:     [XOR_CHECKSUM]    XOR di tutti i byte precedenti

Esempio telemetria:
[0xTH] [0x01] [0x10 0x00] [float T][float Tset][float err][float cmd] [checksum]
```

---

## 5. Next Steps (Fase 2)

Prima di compilare, in Fase 2 creeremo:

1. **sensors.h/c** — Legge temperatura simulata con rumore
2. **pid_controller.h/c** — Algoritmo P, PI, PID
3. **telemetry.h/c** — Codifica/decodifica pacchetti
4. **scheduler.h/c** — Task timer a 100 ms
5. **platform.h/c** — Sleep/timer cross-platform
6. **main.c** — Loop principale che collega tutto

Una volta compilata Fase 2, avremo il firmware che funziona!

---

## 6. Checklist Fase 1

- [x] SRS.md scritto (7 sezioni)
- [x] CMakeLists.txt root + 4 subdirectory
- [x] Struttura directory creata
- [x] Questo README

**Fase 1 completa!** ✅
