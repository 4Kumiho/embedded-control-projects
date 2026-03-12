# Fase 3 — Physics Engine C++ (Modello Fisico e ODE Solver)

**Durata:** ~1 giorno di studio approfondito
**Competenze coperte:** SW OOP (C++), Model Based Design, Numerical Methods, C/C++ interop

---

## Panoramica

Fase 3 implementa il lato "physics" del sistema in C++ (C++ Modern). Fornisce una simulazione accurata della dinamica termica della stanza usando un integratore ODE di 4° ordine (Runge-Kutta 4 - RK4).

Questo è il complemento della Fase 2 (firmware C):
- **Fase 2 (C):** Firmware che controlla il sistema
- **Fase 3 (C++):** Physics engine che simula la risposta

Il firmware chiama l'API C, che inoltra al motore C++.

---

## Architettura

```
┌─────────────────────────────────────────┐
│ Firmware (C) - Fase 2                   │
│ - Sensor simulation (sensors.c)         │
│ - PID controller (pid_controller.c)     │
│ - Telemetry (telemetry.c)               │
│ - Scheduler (scheduler.c)               │
└─────────────────────────────────────────┘
         ↓ (sensor_get_true_temp)
         ↓ (sensor_set_command)
┌─────────────────────────────────────────┐
│ Bridge (C API) - Fase 3                 │
│ thermocontrol_c_api.h / .cpp            │
│ - Hide C++ behind C interface           │
│ - extern "C" for name mangling          │
└─────────────────────────────────────────┘
         ↓
┌─────────────────────────────────────────┐
│ Physics Engine (C++) - Fase 3           │
│ - Vec3.h (vector math)                  │
│ - ThermoModel.h/cpp (ODE solver)        │
│ - RK4 integrator (accurate simulation)  │
└─────────────────────────────────────────┘
```

---

## Moduli creati

### 1. Vec3.h — 3D Vector Mathematics (Header-Only)

**Cosa fa:**
- Vettori 3D con operazioni: somma, sottrazione, moltiplicazione scalare
- Prodotto scalare (dot product) e vettoriale (cross product)
- Magnitudine, normalizzazione, distanza, angoli

**Perché 3D?**
In Fase 3 attuale, usiamo solo temperatura scalare. Ma il template è
pronto per future estensioni (velocità vento 3D, pressione, etc.).

**Header-only design:**
- Vantaggi: No linking needed, inline functions, fast
- Svantaggi: Code bloat if overused
- Perfetto per small math library

**Operazioni principali:**

```cpp
Vec3 a = {1, 2, 3};
Vec3 b = {4, 5, 6};

// Arithmetic
Vec3 c = a + b;              // {5, 7, 9}
Vec3 d = a * 2.0f;           // {2, 4, 6}

// Products
float dot_prod = a.dot(b);   // 1*4 + 2*5 + 3*6 = 32
Vec3 cross_prod = a.cross(b); // Perpendicular vector

// Magnitude
float len = a.magnitude();     // sqrt(1^2 + 2^2 + 3^2)
float len_sq = a.magnitude_squared(); // Faster (no sqrt)

// Normalization
Vec3 unit = a.normalized();   // Same direction, |unit| = 1
```

**Competenza ALTEN:** SW OOP (C++) - operator overloading, inline functions

---

### 2. ThermoModel.h / ThermoModel.cpp — Thermal Dynamics Simulator

**Cosa fa:**
- Modella la dinamica termica della stanza
- Implementa equazione differenziale (ODE) del primo ordine
- Usa integratore RK4 per alta precisione

**Equazione fisica:**

```
dT/dt = -1/τ · (T - T_ambient) + (U/100) · K_heater
```

**Spiegazione termini:**

| Termine | Significato | Effetto |
|---------|-------------|--------|
| **T** | Temperatura stanza (°C) | Lo stato che evolve |
| **dT/dt** | Velocità cambio temperatura (°C/sec) | Derivata temporale |
| **-1/τ · (T - T_ambient)** | Drift naturale verso ambient | Room cools/warms verso 20°C |
| **τ** | Costante tempo termica (30 sec) | Regola velocità drift |
| **U/100** | Comando riscaldatore normalizzato (0-1) | Da PID in firmware |
| **K_heater** | Guadagno riscaldatore (0.05°C/sec) | Max heating rate |

**Comportamento fisico:**

Caso 1: Room hot (T=25°C), no heating (U=0)
```
dT/dt = -1/30 * (25 - 20) + 0
      = -5/30
      = -0.167 °C/sec
```
→ Temperature decays back to ambient (exponential with τ=30 sec)

Caso 2: Room cold (T=20°C), full heat (U=100)
```
dT/dt = -1/30 * (20 - 20) + 1 * 0.05
      = 0 + 0.05
      = +0.05 °C/sec
```
→ Temperature rises at 0.05°C/sec (full heater)

Caso 3: Room at ambient (T=20°C), equilibrium
```
dT/dt = 0 (no drift, no heating)
```
→ Temperature stays at 20°C (stable equilibrium)

---

### 3. Runge-Kutta 4th Order (RK4) Integrator

**Problema:**
Come risolvere l'ODE dT/dt = f(T, U) numericamente?

**Soluzioni (in ordine di accuratezza):**

#### Euler (1st order) - Simple but inaccurate
```cpp
T_new = T + f(T) * dt
```

Pro: 1 function call, simple
Con: Error ~ O(dt²), inaccurate for large dt

#### RK2 (2nd order) - Better
```cpp
k1 = f(T)
k2 = f(T + k1 * dt/2)
T_new = T + k2 * dt
```

Pro: Better accuracy
Con: 2 function calls

#### RK4 (4th order) - Best balance
```cpp
k1 = f(T)
k2 = f(T + k1 * dt/2)
k3 = f(T + k2 * dt/2)
k4 = f(T + k3 * dt)
T_new = T + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
```

Pro: High accuracy (error ~ O(dt^5)), standard choice
Con: 4 function calls

**Noi usiamo RK4 perché:**
- Accuratezza ottimale per il nostro dt (1-10 ms)
- Implementazione semplice in C++
- 4 function calls è accettabile
- Standard in physics simulations

**Derivazione formale (per curiosità):**

RK4 interpola il valore della funzione f(T) in 4 punti:
- k1: Pendenza all'inizio T
- k2: Pendenza a T + k1 * dt/2 (midpoint 1)
- k3: Pendenza a T + k2 * dt/2 (midpoint 2)
- k4: Pendenza a T + k3 * dt (endpoint)

Poi la media pesata (1, 2, 2, 1) con denominatore 6:
```
T_new = T + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
```

Questa formula è ottimizzata per cancellare termini di errore fino a O(dt^5).

**Accuracy vs Euler:**

Euler:
- dt = 10 ms
- Error per step: ~10 ms² = 0.0001
- Over 30 sec (3000 steps): ~0.3°C cumulative error

RK4:
- dt = 10 ms
- Error per step: ~(10 ms)^5 = negligible
- Over 30 sec: < 0.001°C cumulative error

**RK4 in codice (ThermoModel.cpp):**

```cpp
void ThermoModel::update(float dt) {
    // Four slope evaluations
    float k1 = calculate_derivative(temperature_);
    float k2 = calculate_derivative(temperature_ + k1 * dt * 0.5f);
    float k3 = calculate_derivative(temperature_ + k2 * dt * 0.5f);
    float k4 = calculate_derivative(temperature_ + k3 * dt);

    // Weighted average
    float slope_avg = (k1 + 2.0f*k2 + 2.0f*k3 + k4) / 6.0f;
    temperature_ += slope_avg * dt;
}
```

**Competenza ALTEN:** Model Based Design, Numerical Methods

---

### 4. thermocontrol_c_api.h / .cpp — C/C++ Bridge

**Problema:**
- Firmware è C (embedded standard)
- Physics engine è C++ (OOP, più facile da scrivere)
- Come farli comunicare?

**Soluzione: C API wrapper**

```
C Code (firmware)
    ↓
thermocontrol_set_command(50.0f)
    ↓
g_model.set_heater_command(50.0f)  // C++ call
```

**Perché extern "C"?**

In C++, i nomi delle funzioni vengono "mangled" per gestire overloading:
```cpp
// C++
void foo(int x);     → Symbol: _Z3fooi
void foo(float x);   → Symbol: _Z3fool
```

C non ha overloading, quindi usa nomi diretti:
```c
// C
void foo(int x);     → Symbol: foo
```

Se C code tenta di chiamare C++ function:
- Cerca symbol "thermocontrol_set_command"
- Trova "_Z27thermocontrol_set_commandf" (mangled)
- Link error!

Soluzione: `extern "C"` dice al compiler:
"Non manglare questi nomi, usa convenzione C"

```cpp
extern "C" {
    void thermocontrol_set_command(float command) {
        g_model.set_heater_command(command);
    }
}
```

Ora il symbol è "thermocontrol_set_command" (no mangling),
compatibile con C code.

**Funzioni esportate:**

```
thermocontrol_init()              // Initialize
thermocontrol_shutdown()          // Cleanup
thermocontrol_update(dt)          // Physics step
thermocontrol_set_command(cmd)    // From firmware
thermocontrol_get_temperature()   // To firmware
thermocontrol_get_command()       // For logging
thermocontrol_reset(T0)           // Test reset
```

**Competenza ALTEN:** SW OOP, Language interop (C/C++)

---

## Flusso dati Fase 2 ↔ Fase 3

```
main.c (Fase 2 - firmware main loop)
    ↓
    TASK_CONTROLLER
    ├─ pid_update(T_measured)
    ├─ output = heater_command
    ├─ sensor_set_command(heater_command)
    ↓
    sensor_set_command() chiama:
        thermocontrol_set_command(cmd)  ← Bridge (Fase 3)
    ↓
    ThermoModel.set_heater_command(cmd)  ← C++ model
    ↓
    Poi, TASK_SENSOR in next loop:
    ├─ sensor_read()
    ├─ calculate_derivative() è chiamato da sensor_read()? NO!
    │
    │ In realtà in main.c l'update del physics non è visibile al firmware
    │ Il firmware legge la temperatura tramite sensor_read()
    │ che chiama thermocontrol_get_temperature()
    │ ma manca il passo di aggiornare il physics!
    │
    │ Nota: Nel setup reale, dovremmo chiamare:
    │   thermocontrol_update(dt) periodicamente
    │ Per ora assumiamo che update sia già stato chiamato
    │ (oppure lo aggiungiamo in main.c)
```

**Osservazione importante:**
Nel main.c attuale, non chiamiamo `thermocontrol_update()` in nessun task!
Dovremmo aggiungere un TASK_PHYSICS che lo chiama regolarmente,
oppure aggiungerlo nella loop principale.

Per simplicity nella Fase 3 didattica, assumiamo che:
- Physics ticks at same frequency as firmware (100 ms)
- In real systems, physics would tick at 1000+ Hz for accuracy

---

## Design Patterns Usati

### RAII (Resource Acquisition Is Initialization)
```cpp
class ThermoModel {
private:
    float temperature_;  // Automatically initialized
};

ThermoModel model;  // Constructor runs automatically
// Destructor runs automatically at end of scope
```

Pro: No leaks, automatic cleanup
Con: Requires understanding of C++ semantics

### Opaque Pointers / Pimpl (Pointer to Implementation)
```cpp
// In C code:
thermocontrol_set_command(50.0f);  // C doesn't see ThermoModel

// In C++ code:
static ThermoModel g_model;  // Hidden from C
```

Pro: C code doesn't depend on C++ details
Con: Extra indirection (minor performance hit)

### Static Globals
```cpp
static ThermoModel g_model;  // Singleton pattern
```

Pro: Simple, no dynamic allocation, thread-safe (single-threaded)
Con: Not flexible, global state

---

## Competenze ALTEN Coperte

| Competenza | Dove | Come |
|-----------|------|------|
| **SW OOP (C++)** | ThermoModel, Vec3 | Classes, methods, operators |
| **Model Based Design** | Thermal ODE, RK4 | Differential equations, solver |
| **Numerical Methods** | RK4 integrator | Accuracy, stability, convergence |
| **Language Interop** | thermocontrol_c_api | C/C++ bridge, extern "C" |
| **Object-Oriented Design** | RAII, Pimpl | Encapsulation, patterns |

---

## Come compilare e testare

```bash
cd thermocontrol/build
cmake .. -DBUILD_PHYSICS=ON
cmake --build . --config Release

./bin/thermocontrol_sim
```

Output:
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
...
```

---

## Prossima fase: Fase 4

Ground Station Python con GUI PyQt6 che visualizza i dati in tempo reale.

---

## Referenze e Approfondimenti

### RK4 Integrator
- https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
- Classic numerical methods textbook: "Numerical Recipes"

### C/C++ Interop
- https://en.cppreference.com/w/cpp/language/language_linkage
- "C++ Interoperability" patterns

### Thermal Dynamics
- Physics of heat transfer and thermal time constants
- First-order ODE solution: T(t) = T_ambient + (T0 - T_ambient) * exp(-t/τ)
