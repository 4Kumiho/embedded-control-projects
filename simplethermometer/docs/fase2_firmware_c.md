# Fase 2 — Firmware C (Sensor & Serial I/O)

**Durata:** ~1 ora (legge e comprende il codice)
**Competenze coperte:** SW Embedded C, Sensor I/O, Serial Communication

---

## Panoramica

Fase 2 implementa il firmware C che:
1. Legge da sensore simulato
2. Trasmette via seriale
3. Esegue loop principale ogni 500ms

**Novità:** Ogni singola riga ha un commento spiegando cosa fa.

---

## Competenza ALTEN Coperta

✅ **SW Embedded (C)** — Firmware con sensor I/O e serial communication

---

## File Creati

### 1. sensor.h / sensor.c — Sensor Module

**Cosa fa:**
- Simula un sensore di temperatura
- Genera oscillazione sinusoidale (±5°C)
- Aggiunge rumore gaussiano (±0.5°C)
- Applica filtro passa-basso (first-order)
- Clamp tra 15-35°C

**Funzioni:**

```c
void sensor_init(void);           // Seed random number generator
float sensor_read_temperature(void); // Leggi temperatura corrente
```

**Come funziona:**

```
┌─────────────────────────────────────┐
│ 1. Generate base temp (sine wave)   │
│    Base = 25°C + 5°C * sin(angle)   │
│                                      │
│ 2. Add random noise (±0.5°C)        │
│    Temp_noisy = Base + Noise        │
│                                      │
│ 3. Apply low-pass filter            │
│    Temp_filtered = 0.1*new + 0.9*old│
│    (makes response slow/realistic)  │
│                                      │
│ 4. Clamp to [15, 35]°C              │
│    if T < 15: T = 15                │
│    if T > 35: T = 35                │
│                                      │
│ → Return filtered temperature       │
└─────────────────────────────────────┘
```

**Line-by-line comments:**
- Tutti i commenti spiegano esattamente cosa fa quella riga
- Variabili globali commentate
- Formule matematiche spiegate

**Esempio:**
```c
/* Incremento counter (rappresenta ogni lettura 500ms) */
g_time_counter++;

/* Converto counter a angolo in radianti per onda sinusoidale */
float angle = (2.0f * 3.14159f * (float)g_time_counter) / 120.0f;
```

### 2. serial.h / serial.c — Serial Module

**Cosa fa:**
- Formatta temperatura come stringa "TEMP:XX.X°C\r\n"
- Trasmette via stdout (simula seriale)
- In vero HW userebbe UART driver

**Funzioni:**

```c
void serial_init(void);                    // Init serial port (no-op in sim)
void serial_send_temperature(float temp);  // Transmit formatted string
```

**Format:**
```
TEMP:22.3°C
TEMP:24.5°C
TEMP:21.8°C
```

**Parametri:**
- Baud: 115200 (in vero sistema)
- Format: `TEMP:%.1f°C\r\n`
- Terminazione: `\r\n` (CR+LF)

### 3. main.c — Main Loop

**Cosa fa:**
```c
while (1) {
    temperature = sensor_read_temperature();
    serial_send_temperature(temperature);
    usleep(500000);  // Sleep 500ms
}
```

**Cycle time:** ~500ms per iteration

---

## Spiegazione Dettagliata

### Sensore (First-Order Model)

Il sensore simula un sistema termico reale usando filtro passa-basso:

```
Discrete model:
  T[n] = α·T_new + (1-α)·T[n-1]

Con α = 0.1:
  - 10% nuovo valore
  - 90% valore precedente
  - Cambiamenti lenti e realistici
```

**Perché questo è realistico?**
- Veri sensori hanno ritardo termico
- Cambiano lentamente, non istantaneamente
- Temperatura non salta di 5°C tra readings

### Oscillazione e Rumore

```
Temperature = Base + Oscillation + Noise

Base        = 25.0°C (setpoint desiderato)
Oscillation = ±5.0°C (sine wave su 120 readings = 60 secondi)
Noise       = ±0.5°C (random, per realismo)
```

---

## Compilazione

```bash
# Compilare firmware
gcc -o firmware_main firmware/main.c firmware/sensor.c firmware/serial.c -lm

# Eseguire
./firmware_main
```

Output:
```
========================================
SimpleThermometer v1.0
Reading temperature every 500ms
========================================
TEMP:25.3°C
TEMP:24.8°C
TEMP:25.1°C
```

---

## Competenze Demonstrate

1. **C Basics** — struct, functions, arrays
2. **Embedded I/O** — Sensor reading, serial transmission
3. **Control Loop** — Main loop with periodic timing
4. **Simulation** — Realistic sensor model
5. **Code Comments** — Every line explained
6. **Cross-Platform** — usleep() works on Linux/Windows

---

## Prossimo: Fase 3

Python Ground Station:
- Serial reader thread
- PyQt6 GUI
- Real-time plotting

---
