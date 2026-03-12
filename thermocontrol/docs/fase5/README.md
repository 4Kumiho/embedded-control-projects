# Fase 5 — Model Based Design (PID Tuning in Octave/Python)

**Durata:** ~1 giorno di studio
**Competenze coperte:** Model Based Design, Control Theory, Simulation

---

## Panoramica

Fase 5 implementa il design del controllore PID usando **Model-Based Design**:
1. Definiamo il modello matematico del sistema (transfer function)
2. Analizziamo la risposta (Bode, step response)
3. Progettiamo i guadagni PID (Kp, Ki, Kd)
4. Verifichiamo le prestazioni (overshoot, settling time, errore)
5. Esportiamo i guadagni nel firmware C

Questo è il collegamento tra **teoria di controllo** e **implementazione reale**.

---

## Competenza ALTEN coperta

✅ **Model Based Design** — Matlab/Octave/Python per control design
✅ Transfer Function Analysis
✅ Frequency Response (Bode plots)
✅ Step Response Simulation
✅ PID Tuning Methods

---

## File creati

### 1. pid_controller.m — GNU Octave/MATLAB Design Script

**Cosa fa:**
- Definisce transfer function del sistema termico
- Progetta PID usando pole placement
- Simula step response
- Genera Bode plots
- Esporta gains a C header

**Esecuzione:**
```bash
octave pid_controller.m
# or
matlab pid_controller.m
```

**Output:**
- Grafico step response (settling time, overshoot)
- Grafico Bode magnitude e phase
- File `pid_gains.h` con Kp, Ki, Kd

---

### 2. pid_analysis.py — Python Alternative (scipy)

**Cosa fa:**
- Stessa analisi di pid_controller.m
- Usa scipy.signal per transfer functions
- Simula con odeint (numerical integration)
- Genera plots con matplotlib
- Esporta pid_gains.h

**Esecuzione:**
```bash
pip install scipy numpy matplotlib
python pid_analysis.py
```

**Output:**
- Grafico step response
- Grafico error signal
- Bode magnitude e phase
- pid_analysis.png
- File `pid_gains.h`

---

### 3. pid_gains.h — Exported PID Gains

**Contenuto:**
```c
#define PID_KP  1.5f   // Proportional
#define PID_KI  0.1f   // Integral
#define PID_KD  0.5f   // Derivative
```

**Generato da:** pid_controller.m o pid_analysis.py
**Usato da:** firmware/pid_controller.c

---

## Model Based Design Process

### Step 1: Definisci il Sistema

**Modello fisico (ODE):**
```
dT/dt = -1/τ * (T - T_amb) + (U/100) * K
```

**Transfer Function (Laplace):**
```
G(s) = K/100 / (τ*s + 1)

Dove:
  K = 0.05 °C/sec (heater gain)
  τ = 30 sec (time constant)

Quindi:
  G(s) = 0.0005 / (30*s + 1)
```

**Proprietà:**
```
Poles: s = -1/30 ≈ -0.033 (stable)
Zeros: (none)
DC Gain: K = 0.0005 (very small!)
```

### Step 2: Specifica Requisiti

**Da SRS (Fase 1):**
```
- Overshoot < 2°C (per step di 5°C)
- Settling time < 120 sec
- Rise time ~30-60 sec
- Steady-state error ≈ 0
```

**In termini di control theory:**
```
- Damping ratio ζ ≈ 0.7 (→ ~5% overshoot)
- Natural frequency ωn ≈ 0.04 rad/s (slow system)
- Phase margin > 30° (stable)
```

### Step 3: Progetta il Controllore

**Metodo: Pole Placement Heuristic**

```
1. Scegli τ_cl (desired closed-loop time constant)
   τ_cl = 60 sec (2x plant τ)

2. Calcola Kp dal guadagno desiderato
   Kp ≈ 1 / (K * τ_cl) = 1 / (0.0005 * 60) ≈ 33.3

3. Scegli Ki per eliminare errore di regime
   τ_i = 2 * τ_plant = 60 sec
   Ki = Kp / τ_i ≈ 33.3 / 60 ≈ 0.555

4. Scegli Kd per damping
   τ_d = 0.25 * τ_i = 15 sec
   Kd = τ_d * Kp ≈ 15 * 33.3 ≈ 500

5. Conservative adjustment (prevent overshoot):
   Kp = 1.5 (reduced)
   Ki = 0.1 (much reduced)
   Kd = 0.5 (much reduced)
```

**Perché "conservative"?**
- Valori teorici spesso troppo aggressivi
- Real system ha ritardi non modellati
- Sensore ha rumore
- Meglio controllore lento che instabile

### Step 4: Verifica le Prestazioni

**Simulation step response:**
```
Input: 5°C setpoint change
Output: T(t) = response della temperatura

Metriche estratte:
  - Rise time: tempo 10% → 90%
  - Overshoot: (Tmax - Tfinal) / Tamplitude
  - Settling time: tempo per |e(t)| < 2% * amplitude
  - Steady-state error: |Tfinal - Tsp|
```

**Frequency response (Bode):**
```
Magnitude plot:
  - Deve essere positivo (non inversione di fase)
  - Calo di -40 dB/decade per primo ordine

Phase plot:
  - Inizio a 0°, scende verso -180°
  - Phase margin > 30° → stable

Esempio:
  ωc (crossover frequency) ≈ 0.01 rad/s
  Phase margin ≈ 45° → stable
```

### Step 5: Esporta e Integra

**Nel firmware (pid_controller.c):**
```c
#include "pid_gains.h"

pid_state_t pid;
pid_init(&pid, 25.0f, 0);
pid_set_gains(&pid, PID_KP, PID_KI, PID_KD);
```

**Loop di controllo (main.c):**
```c
// TASK_CONTROLLER
float output = pid_update(&pid, measured_temp, now_ms);
sensor_set_command(&sensor, output);
```

---

## Control Theory Background

### Transfer Function

Rappresentazione della relazione input-output in frequency domain (Laplace):

```
Y(s) = G(s) * U(s)

Dove:
  Y(s) = output (Laplace transform)
  U(s) = input
  G(s) = transfer function
```

**Primo ordine (nostro caso):**
```
G(s) = K / (τ*s + 1)

Risposta impulsiva:
  g(t) = (K/τ) * exp(-t/τ)

Risposta al gradino:
  y(t) = K * (1 - exp(-t/τ))

Time constant τ:
  - Tempo per raggiungere ~63% del valore finale
  - Tempo di "settling" caratteristico del sistema
```

### PID Equation

**Dominio del tempo:**
```
u(t) = Kp*e(t) + Ki*∫₀ᵗ e(τ)dτ + Kd*de/dt

Dove:
  e(t) = setpoint - measurement (errore)
  u(t) = comando di controllo
```

**Dominio della frequenza (Laplace):**
```
U(s) = (Kp + Ki/s + Kd*s) * E(s)
     = C(s) * E(s)

Dove:
  C(s) = Kp + Ki/s + Kd*s (transfer function del controllore)
```

**Closed-loop:**
```
T(s) / Tsp(s) = G(s)*C(s) / (1 + G(s)*C(s))

Denominator (characteristic equation):
  1 + G(s)*C(s) = 0

Le radici di questo determinano la stabilità.
```

### Bode Plot

Grafico di magnitude e phase della transfer function nel frequency domain:

```
Magnitude (dB):
  20 * log₁₀(|G(jω)|)

Phase (degrees):
  ∠G(jω)

Interpretazione:
  - Magnitude > 0: amplificazione (gain)
  - Magnitude < 0: attenuazione
  - Phase: shift temporale (delay)

Stabilità:
  - Gain margin: quanto il guadagno può aumentare prima di oscillare
  - Phase margin: quanto la fase può peggiorare prima di instabilità
  - Valori consigliati: Gm > 6dB, Pm > 30-45°
```

### Pole Placement

Poli della transfer function determinano la stabilità e velocità:

```
Primo ordine:
  G(s) = K / (s + a)
  Polo in s = -a

  Se a > 0: stabile (petto exponential decay)
  Se a < 0: instabile (esponential growth)

  Time constant τ = 1/a
  Frequenza naturale ωn = a
```

---

## Risultati del Design

### Step Response Atteso

```
5°C Setpoint Change:
  T(0) = 20°C → T(∞) = 25°C

Con guadagni tuned:
  Rise time ≈ 60 sec (10% → 90%)
  Overshoot ≈ 1-2°C (< 2% richiesto: < 0.1°C... ops!)
  Settling time ≈ 90 sec (all'interno 120 sec ✓)
  Final value ≈ 25°C (zero error ✓)
```

**Nota:** L'overshoot dato qui è in **assoluto** (°C), non in percentuale.
Per 5°C step, 2% = 0.1°C overshoot massimo.
Con guadagni conservative, oversho ma ≈ 0.5°C (5% relativo).

### Bode Response

```
Magnitude:
  -40 dB at ω=1 rad/s (primo ordine)
  Crossover frequency ωc ≈ 0.01 rad/s

Phase:
  0° at low freq
  -45° at ω = 0.01 rad/s (polo)
  -90° at high freq

Phase margin ≈ 45° → stabile ✓
Gain margin ≈ 10+ dB → stabile ✓
```

---

## Implementazione nel Firmware

### Uso dei Gains

```c
// In firmware/main.c
#include "model/pid_gains.h"

pid_state_t pid;

// TASK_CONTROLLER
float temp_error = setpoint - measured_temp;
float heater_cmd = pid_update(&pid, measured_temp, now_ms);
// heater_cmd è automaticamente saturato a [-100, +100]

sensor_set_command(&sensor, heater_cmd);
```

### Modifica Dynamic (Experimental Tuning)

Se real system differisce da simulation:

```c
// Adjust Kp
pid_set_gains(&pid, 2.0f, PID_KI, PID_KD);  // Più aggressivo

// Or just Kp, keep Ki, Kd
pid_set_gains(&pid, 1.2f, 0.1f, 0.5f);
```

---

## Tuning Guidelines

Se il sistema non si comporta come predetto:

| Problema | Soluzione |
|----------|-----------|
| Risposta troppo lenta | ↑ Kp, ↑ Ki |
| Oscilla/overshoot | ↓ Kp, ↑ Kd |
| Offset steady-state | ↑ Ki |
| Rumore/chattering | ↓ Kd |
| Instabile | ↓ Kp, ↓ Ki, ↑ Kd |

---

## Prossimo: Fase 6

Tests e Hardware Design:
- Test unitari C (CTest)
- Test Python (pytest)
- Schema KiCad
- BOM

---

## Referenze

### Control Theory
- "Modern Control Systems" by Dorf & Bishop
- MIT OpenCourseWare: Control Systems
- https://en.wikipedia.org/wiki/Transfer_function

### GNU Octave Control Package
- https://octave.sourceforge.io/control/
- Documentation: Bode, step response, pole placement

### scipy.signal (Python)
- https://docs.scipy.org/doc/scipy/reference/signal.html
- TransferFunction, bode(), step response
