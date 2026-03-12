# Fase 2 — Firmware C (Embedded Development)

**Durata:** ~2 giorni di studio approfondito
**Competenze coperte:** SW Embedded (C), FW Design, protocolli, real-time systems

---

## Panoramica

Fase 2 implementa il firmware simulato (codice C "bare metal") che controlla il sistema di temperatura. È il cuore della simulazione e copre tutti gli aspetti dell'embedded firmware reale:

1. **Platform abstraction** — Astrazione OS-indipendente
2. **Sensor drivers** — Lettura sensori con modello fisico
3. **Control loop** — Algoritmo PID
4. **Communication** — Protocollo binario
5. **Scheduling** — Real-time task management
6. **Main loop** — Coordinamento di tutto

---

## Moduli creati

### 1. platform.h/c — Platform Abstraction Layer

**Cosa fa:**
- Astrae funzioni di timing da Windows/Linux
- Fornisce timer ad alta precisione
- Sleep cross-platform

**Competenza ALTEN:** "Programmazione su architetture embedded"

**Concetti chiave:**

| Concetto | Spiegazione |
|----------|-------------|
| **Abstraction layer** | Separa codice applicativo da dipendenze HW/OS |
| **Preprocessor directives** | `#ifdef _WIN32` per compilazione condizionale |
| **High-resolution timers** | QueryPerformanceCounter (Windows) vs clock_gettime (Linux) |
| **Timer precision** | Misurazione in microsecond (≈µs) per accuratezza |

**Funzioni principali:**

```c
int platform_init(void)              // Init timers
void platform_timer_start(timer*)    // Start timer
uint32_t platform_timer_elapsed_ms() // Elapsed time
void platform_sleep_ms(ms)           // Sleep blocking
time_t platform_get_time_unix()      // Unix timestamp
```

**Lezione didattica:**

Nel firmware embedded reale:
- Non c'è Windows/Linux
- Useremmo timer HW dell'MCU (STM32, Arduino)
- Esempio STM32: `TIM2_Init()`, `TIM2_GetCount()`
- Noi qui imitiamo questo comportamento su PC

Se dovessi portare questo codice su un vero microcontrollore:
1. Sostituisci `platform.c` con versione per MCU target
2. Rest del codice rimane identico!
3. Questo è il potere dell'abstraction layer

---

### 2. sensors.h/c — Temperature Sensor Simulation

**Cosa fa:**
- Simula sensore temperatura con rumore realistica
- Implementa modello termico (equazione differenziale)
- Genera rumore Gaussiano

**Competenza ALTEN:** "FW (driver sensori)", "Modello fisico"

**Modello fisico (equazione differenziale):**

```
dT/dt = -1/τ · (T - T_ambient) + (U/100) · K_heater

Dove:
- τ = costante tempo termico (30 sec)
- T = temperatura stanza
- T_ambient = temperatura ambiente (20°C)
- U = comando riscaldatore (0-100%)
- K_heater = coefficiente riscaldamento (0.05°C/sec)
```

**Soluzione numerica:**

Usiamo integrazione Euler (semplice, adatto per aggiornamento ogni 100ms):

```c
dT = (delta_t_ambient + delta_t_heater) * dt
T_new = T_old + dT
```

**Rumore Gaussiano (Box-Muller):**

```c
float noise = gaussian_random() * SENSOR_NOISE_SIGMA
// noise ~ N(0, σ) dove σ = 0.05°C
```

**Funzioni principali:**

```c
int sensor_init(sensor_state_t*)              // Init sensor
int sensor_read(sensor_state_t*, time, out*)  // Leggi temperatura + rumore
void sensor_set_command(sensor_state_t*, cmd) // Heater command
float sensor_get_true_temp(sensor_state_t*)   // Leggi valore vero (no rumore)
```

**Lezione didattica:**

Sensore reale (BMP280, DHT22):
```
I2C (2 wire) → Microcontrollore → ADC conversion → temperatura
```

Nel nostro codice:
```
Modello fisico (C++) → sensor_read() → firmware (C)
```

Protocollo identico! Il firmware non sa se legge sensore vero o simulato.

---

### 3. pid_controller.h/c — PID Feedback Control

**Cosa fa:**
- Implementa controllo PID (Proporzionale + Integrale + Derivativo)
- Mantiene temperatura al setpoint
- Senza oscillazioni eccessive

**Competenza ALTEN:** "Model Based Design", "Control Systems"

**Equazione PID:**

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de/dt

Dove:
- u(t) = output (comando heater, -100 a +100)
- e(t) = error = setpoint - measurement
- Kp = gain proporzionale (risposta veloce)
- Ki = gain integrale (elimina errore di regime)
- Kd = gain derivativo (damping, riduce overshoot)
```

**I tre termini:**

| Termine | Significato | Effetto | Problema |
|---------|-------------|--------|----------|
| **P** | Reazione proporzionale all'errore | Risposta veloce | Overshooting |
| **I** | Accumula errore passato | Elimina offset | Oscillazione se troppo grande |
| **D** | Reazione alla velocità di cambiamento | Damping | Amplifica rumore |

**Implementazione (tempo discreto):**

```c
// 1. Errore
error = setpoint - measurement

// 2. P term
p_term = Kp * error

// 3. I term (integrazione numerica)
integral_sum += Ki * error * dt
integral_sum = clamp(integral_sum, -500, 500)  // Anti-windup
i_term = integral_sum

// 4. D term (velocità di errore)
de_dt = (error - error_prev) / dt
d_term = Kd * de_dt

// 5. Somma e saturazione
output = p_term + i_term + d_term
output = clamp(output, -100, 100)
```

**Funzioni principali:**

```c
int pid_init(pid_state_t*, setpoint, time)    // Init controller
float pid_update(pid_state_t*, measure, time)  // Calculate output
void pid_set_setpoint(pid_state_t*, sp)        // Change setpoint
void pid_set_gains(pid_state_t*, Kp, Ki, Kd)  // Tune gains
void pid_reset(pid_state_t*, time)             // Clear history
```

**Lezione didattica:**

Processo di tuning (Ziegler-Nichols, Octave):
1. Misura risposta del sistema (step input)
2. Identifica parametri critici
3. Calcola Kp, Ki, Kd in Octave (Fase 5)
4. Testa su sistema reale
5. Itera fino a soddisfare specifiche

I nostri guadagni (da Octave):
- Kp = 1.5 (proporzionale, veloce ma non troppo)
- Ki = 0.1 (lento, per eliminare offset)
- Kd = 0.5 (damping, riduce overshoot)

---

### 4. telemetry.h/c — Binary Protocol

**Cosa fa:**
- Codifica pacchetti in formato binario
- Verifica integrità con XOR checksum
- Permette comunicazione firmware ↔ GUI

**Competenza ALTEN:** "Protocolli di comunicazione", "Verifica e Validazione"

**Formato packet:**

```
Byte 0:    [0x54]          (magic byte = 'T')
Byte 1:    [0x01]          (type = telemetry)
Byte 2-3:  [0x10 0x00]     (length = 16, little-endian)
Byte 4-7:  [float]         (temperature)
Byte 8-11: [float]         (setpoint)
Byte 12-15: [float]        (error)
Byte 16-19: [float]        (command)
Byte 20:   [checksum]      (XOR di tutti i byte precedenti)
```

**Dimensione:** 21 byte totali

**Checksum XOR:**

```c
checksum = 0
for each byte in frame (except checksum):
    checksum ^= byte
```

Pro:
- Fast (una operazione per byte)
- Semplice (poche linee di codice)
- Detect singoli bit errors

Contro:
- Non detect multi-bit errors correlati

Per reliability maggiore useremmo CRC32 (ma overkill qui).

**Funzioni principali:**

```c
int telemetry_encode(packet*, frame*)  // Pacchetto → bytes
int telemetry_decode(frame*, packet*)  // bytes → pacchetto
int telemetry_verify_checksum(frame*)  // Verifica integrità
```

**Lezione didattica:**

Protocolli reali (CAN, Modbus, proprietary):
```
Magic byte     → Identificazione frame
Type field     → Quale tipo di dato
Length field   → Dimensione payload
Payload        → Dati
Checksum/CRC   → Rilevamento errori
```

Nel nostro sistema:
- Firmware → serializza a binario
- Invia a GUI (socket in produzione)
- GUI → deserializza e visualizza

---

### 5. scheduler.h/c — Task Scheduling

**Cosa fa:**
- Gestisce 3 task periodici (sensore, PID, telemetria)
- Determinismo temporale (task eseguiti nei tempi corretti)
- Senza OS/threads (cooperativo)

**Competenza ALTEN:** "Real-time systems", "Embedded patterns"

**Task registrati:**

| Task | Periodo | Funzione |
|------|---------|----------|
| TASK_SENSOR | 100 ms | Leggi temperatura |
| TASK_CONTROLLER | 100 ms | Aggiorna PID |
| TASK_TELEMETRY | 100 ms | Invia telemetria |

**Main loop pattern:**

```c
while (1) {
    time = get_current_time()
    task = scheduler_update(time)

    if (task == TASK_SENSOR) {
        sensor_read()
    } else if (task == TASK_CONTROLLER) {
        pid_update()
    } else if (task == TASK_TELEMETRY) {
        telemetry_send()
    } else {
        // Nessun task due, posso dormire
        sleep(1)
    }
}
```

**Timing accuracy:**

- Se scheduler_update() è chiamato frequentemente (ogni 1-10 ms), timing è accurato
- Se chiamato raramente (ogni 1 secondo), task saranno in ritardo
- Nel nostro main.c, loop gira ~10,000 volte/sec → timing accurato

**Funzioni principali:**

```c
int scheduler_init(scheduler*, start_time)    // Init
int scheduler_register(scheduler, id, period) // Register task
task_id_t scheduler_update(scheduler, time)   // Query task due
void scheduler_set_enabled(scheduler, id, en) // Enable/disable
```

**Lezione didattica:**

RTOS vs bare metal:
- **RTOS** (FreeRTOS, VxWorks): interrupt-driven, preemptive, complex
- **Bare metal** (nostro): polling, cooperative, simple

Su MCU reale useremmo:
```c
// Timer interrupt ogni 10 ms
void TIM2_IRQHandler(void) {
    scheduler_update(current_tick);
    HAL_IncTick();
}
```

Noi usimiamo polling (simpler for simulation).

---

### 6. main.c — Firmware Entry Point

**Cosa fa:**
- Inizializza tutti i moduli
- Implementa main control loop
- Raccoglie statistiche

**Main loop flow:**

```
┌─────────────────────┐
│ Inizializzazione    │
├─────────────────────┤
│ platform_init()     │
│ sensor_init()       │
│ pid_init()          │
│ scheduler_init()    │
│ scheduler_register()│
└─────────────────────┘
         ↓
    ┌────────────────────────────┐
    │   Main Control Loop        │
    ├────────────────────────────┤
    │ get time from platform     │
    │ ask scheduler which task   │
    │ execute task function      │
    │ repeat ~10k times/sec      │
    └────────────────────────────┘
         ↓
   ┌──────────────────┐
   │ After 30 sec:    │
   │ Print stats      │
   │ Exit gracefully  │
   └──────────────────┘
```

**Tre task implementati:**

1. **task_sensor_read():**
   - Chiama `sensor_read()`
   - Legge temperatura con rumore
   - Conta statistiche

2. **task_controller_update():**
   - Legge ultima temperatura da sensor
   - Chiama `pid_update()`
   - Applica comando a sensor (feedback loop!)

3. **task_telemetry_send():**
   - Raccoglie stato (T, setpoint, error, cmd)
   - Codifica in frame binario
   - (In prod: invia a GUI)

---

## Flusso dati end-to-end

```
┌────────────────────────────────────────────────────┐
│ MAIN LOOP (ogni 100 ms)                            │
└────────────────────────────────────────────────────┘
         ↓
    ┌────────────────────────────┐
    │ TASK_SENSOR (t = 0, 100,   │
    │             200, 300, ...) │
    └────────────────────────────┘
    │ sensor_read()
    │   ├─ Get current time
    │   ├─ Apply thermal model (dT/dt)
    │   ├─ Add Gaussian noise
    │   └─ Return T_measured
    ↓
    ┌────────────────────────────┐
    │ TASK_CONTROLLER            │
    │ (same time)                │
    └────────────────────────────┘
    │ pid_update(T_measured)
    │   ├─ Calculate error = setpoint - T_measured
    │   ├─ P term = Kp * error
    │   ├─ I term += Ki * error * dt
    │   ├─ D term = Kd * de/dt
    │   ├─ Sum: output = P + I + D
    │   ├─ Saturate [-100, +100]
    │   └─ Return heater_command
    │
    │ sensor_set_command(heater_command)
    │   └─ Store in sensor for next physics update
    ↓
    ┌────────────────────────────┐
    │ TASK_TELEMETRY            │
    │ (same time)                │
    └────────────────────────────┘
    │ telemetry_encode()
    │   └─ Pack T, Tset, error, cmd into binary frame
    │
    │ (In real system: send frame to GUI)
    │ (In simulation: just count it)
    ↓
    ┌────────────────────────────┐
    │ Repeat next 100 ms         │
    └────────────────────────────┘

LOOP CLOSES:
    sensor_read() → physics update → T increases
    → pid_update() → error decreases → command decreases
    → physics stabilizes → cycle repeats
```

---

## Competenze ALTEN coperte

| Competenza | Dove | Come |
|-----------|------|------|
| **SW Embedded (C)** | platform, sensors, pid, telemetry | Codice C bare metal |
| **FW** | sensors, pid | Driver e algoritmo |
| **Protocolli** | telemetry | Formato binario con checksum |
| **Real-time** | scheduler, main | Task timing deterministico |
| **Modello fisico** | sensors | Equazione differenziale |
| **Control** | pid_controller | Algoritmo PID tuned |

---

## Come compilare

```bash
cd thermocontrol/build
cmake .. -DBUILD_PHYSICS=ON -DBUILD_TESTS=ON
cmake --build . --config Release

# Run firmware simulator
./bin/thermocontrol_sim
```

Expected output:
```
=== ThermoControl Firmware v1.0 ===
Initializing platform...
Initializing sensor...
Initializing PID controller...
Initializing scheduler...
Registering tasks...
Initializing telemetry...

=== Initialization complete ===
Starting main control loop...

Time:     0 ms | T=20.00°C | Tset=25.00°C | err=-5.00°C | cmd=0.0% | iter=...
Time:  5000 ms | T=21.45°C | Tset=25.00°C | err=-3.55°C | cmd=28.3% | iter=...
Time: 10000 ms | T=23.12°C | Tset=25.00°C | err=-1.88°C | cmd=12.5% | iter=...
...
Time: 30000 ms | T=24.98°C | Tset=25.00°C | err=+0.02°C | cmd=0.1% | iter=...

=== Firmware Statistics ===
Total runtime: 30000 ms
Sensor reads: 300 (10.0 Hz)
PID updates: 300 (10.0 Hz)
Telemetry sends: 300 (10.0 Hz)
Main loop iterations: ...
```

---

## Key Takeaways

1. **Abstraction layers** — Separano HW-specific da applicativo
2. **Modular design** — Ogni modulo fa una cosa, fa bene
3. **Deterministic timing** — Scheduler ensures tasks run on schedule
4. **Feedback control** — PID chiude il loop (sensore → output → sensore)
5. **Binary protocols** — Efficiente, checksummed, parseable
6. **Simulation** — Firmware test senza HW reale

---

## Prossimo: Fase 3

Physics Engine C++ che simula il modello termico con integratore RK4 ad alta precisione.

Poi Fase 4: Ground Station Python che visualizza i dati real-time.
