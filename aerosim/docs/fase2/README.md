# Fase 2 — SW Embedded (C) e Firmware

## Cosa abbiamo fatto

Nella Fase 2 abbiamo implementato il **Firmware Simulator** in C, coprendo due competenze ALTEN:
- **SW Embedded**: programmazione C su architetture embedded
- **FW**: driver sensori, protocolli di comunicazione, schedulazione

---

## File creati

| File | Ruolo |
|------|-------|
| `firmware/platform.h` | Astrazione OS (tempo, sleep) |
| `firmware/sensors.h/c` | Driver IMU, GPS, barometro simulati |
| `firmware/scheduler.h/c` | Schedulatore ciclico dei task |
| `firmware/telemetry.h/c` | Protocollo binario + scrittura file |
| `firmware/main.c` | Entry point, registrazione task |

---

## Concetto 1: Scheduler ciclico

### Cos'è
Nei firmware embedded il codice non usa `thread` o `sleep` come nelle app desktop. Invece usa un **timer hardware** che genera un interrupt ogni N millisecondi. Ogni interrupt incrementa un contatore (`tick`), e il main loop controlla quali task sono scaduti.

### Come funziona il nostro

```c
// Ogni task ha un periodo e un timestamp dell'ultima esecuzione
typedef struct {
    uint32_t  period_ms;
    uint32_t  last_run_ms;
    void    (*callback)(void);
} Task;

// Nel loop: se (ora - ultima_esecuzione) >= periodo -> esegui
void scheduler_tick(void) {
    uint32_t now = platform_get_tick_ms();
    for (int i = 0; i < s_task_count; i++) {
        if ((now - s_tasks[i].last_run_ms) >= s_tasks[i].period_ms) {
            s_tasks[i].callback();
            s_tasks[i].last_run_ms = now;
        }
    }
}
```

### Trucco: wrap-around del contatore
La sottrazione `now - last_run` usa interi **unsigned** a 32 bit. Questo gestisce automaticamente l'overflow del contatore dopo ~49 giorni senza if speciali.

### Task registrati
| Task | Periodo | Frequenza | Motivo |
|------|---------|-----------|--------|
| `imu_read` | 10 ms | 100 Hz | IMU ha bisogno di alta frequenza per stima assetto |
| `gps_read` | 200 ms | 5 Hz | Il GPS aggiorna lentamente |
| `telemetry` | 100 ms | 10 Hz | Bilanciamento dati/banda |
| `watchdog` | 500 ms | 2 Hz | Controllo periodico sistema |

---

## Concetto 2: Rumore gaussiano (Box-Muller)

I sensori reali **non** restituiscono valori perfetti. Ogni misura ha un errore stocastico modellato come distribuzione gaussiana N(μ, σ).

### La formula Box-Muller
Genera un campione gaussiano da due numeri uniformi:
```
z = sqrt(-2 · ln(u1)) · cos(2π · u2)
```

### Implementazione
```c
static double gaussian(double mean, double std) {
    double u1 = (double)rand() / RAND_MAX;  // uniforme in (0,1]
    double u2 = (double)rand() / RAND_MAX;
    double z  = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
    return mean + std * z;
}
```

### Parametri di rumore scelti
| Sensore | Grandezza | σ (rumore) | Riferimento reale |
|---------|-----------|------------|-------------------|
| IMU | Accelerazione | 0.05 m/s² | MPU-6050 datasheet |
| IMU | Giroscopio | 0.02 deg/s | MPU-6050 datasheet |
| GPS | Posizione | 0.000005° ≈ 0.5m | u-blox NEO-M9N |
| GPS | Altitudine | 0.5 m | tipico consumer GPS |
| Baro | Pressione | 0.1 hPa | BMP390 datasheet |

---

## Concetto 3: Protocollo binario di telemetria

### Perché binario e non testo?
Su un link radio a bassa banda (es. 115200 baud) un pacchetto testo come:
```
"ax=0.02,ay=-0.01,az=9.82,gx=0.01,..."  → ~50 byte
```
è il doppio di un pacchetto binario equivalente:
```
[AE][01][18 00][...24 byte float...][CS]  → 29 byte
```

### Struttura del pacchetto
```
┌────────┬────────┬──────────┬──────────────────────┬──────────┐
│ START  │  TYPE  │  LENGTH  │       PAYLOAD         │ CHECKSUM │
│  0xAE  │ 1 byte │  2 byte  │    0-255 bytes        │  1 byte  │
└────────┴────────┴──────────┴──────────────────────┴──────────┘
```

### Checksum XOR
```c
uint8_t checksum = 0;
for (uint16_t i = 0; i < len; i++) {
    checksum ^= payload[i];   // XOR accumulato
}
```
Semplice e veloce, adatto a MCU con CPU limitata. Protocolli più robusti (MAVLink, CAN) usano CRC-16 o CRC-32.

### Serializzazione float IEEE 754
I float non possono essere inviati direttamente come interi — usano `memcpy` per copiare i byte raw:
```c
static void write_float(uint8_t *buf, float val) {
    memcpy(buf, &val, sizeof(float));   // 4 byte esatti, no conversioni
}
```

---

## Concetto 4: platform.h — portabilità

Il codice firmware è pensato per girare su MCU. Per compilarlo su PC usiamo una **astrazione del layer hardware** (HAL):

```
MCU reale:      platform_get_tick_ms() → legge registro SysTick
Su Windows:     platform_get_tick_ms() → chiama GetTickCount()
Su Linux:       platform_get_tick_ms() → chiama clock_gettime()
```

Lo stesso pattern è usato in framework professionali come STM32 HAL, Zephyr RTOS e FreeRTOS.

---

## Come compilare e lanciare

```bash
# Dalla root del progetto
mkdir build && cd build
cmake ..
make firmware_sim

# Esegui
./firmware/firmware_sim
```

Output atteso:
```
=============================================
  AeroSim Firmware Simulator v1.0
=============================================

[SCHEDULER] Inizializzato.
[SCHEDULER] Task 'imu_read'   registrato (periodo: 10 ms)
[SCHEDULER] Task 'gps_read'   registrato (periodo: 200 ms)
[SCHEDULER] Task 'telemetry'  registrato (periodo: 100 ms)
[SCHEDULER] Task 'watchdog'   registrato (periodo: 500 ms)

[TELEM] Inviato: IMU=29 B, GPS=33 B, STATUS=12 B | Alt=0.0m Up=100ms
[WDG]   Heartbeat #1 | Quota: 0.2 m
...

=== Statistiche scheduler ===
Task                 Periodo(ms) Esecuzioni
--------------------------------------------
imu_read                     10        500
gps_read                    200         25
telemetry                   100         50
watchdog                    500         10
```

---

## Prossimo passo: Fase 3

Sviluppo del **Physics Engine in C++**:
- `Drone.cpp` — corpo rigido 6DOF, equazioni di Newton-Euler
- `Environment.cpp` — gravità, vento, resistenza aerodinamica
- Integrazione Runge-Kutta 4° ordine
- Interfaccia C-compatible per il firmware
