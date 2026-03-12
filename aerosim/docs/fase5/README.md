# Fase 5 — Model Based Design (MBD)

## Cosa abbiamo fatto

Nella Fase 5 abbiamo implementato il **modello PID** per il controllo di quota del drone, coprendo la competenza ALTEN:
- **Model Based Design**: modellazione matematica, analisi in frequenza, sintesi del controllore
- Il risultato (i guadagni PID) viene esportato direttamente nel firmware C

---

## File creati

| File | Ruolo |
|------|-------|
| `model/pid_controller.m` | Script Octave completo (Matlab-compatible) |
| `model/pid_analysis.py` | Alternativa Python con numpy + matplotlib |
| `model/pid_gains.h` | Header C generato — usato dal firmware |

---

## Come eseguire

```bash
# Octave
cd model
octave pid_controller.m

# oppure Python
pip install numpy matplotlib
python pid_analysis.py
```

---

## Concetto 1: Modello del sistema (Plant)

Il **plant** è il sistema fisico che vogliamo controllare: la dinamica verticale del drone.

### Linearizzazione attorno all'hovering

Il drone in hover ha spinta T₀ = m·g. Definiamo la **deviazione di spinta** u = T - T₀ (piccola perturbazione attorno all'equilibrio).

Le equazioni del moto diventano:
```
m · z'' = u - b · z'

dove:
  z  = quota [m]
  u  = deviazione spinta [N]
  m  = 0.8 kg
  b  = 0.25 N·s/m  (smorzamento aerodinamico)
```

### Funzione di trasferimento

Applicando la trasformata di Laplace (con condizioni iniziali nulle):
```
m · s²·Z(s) = U(s) - b·s·Z(s)

Z(s)/U(s) = G(s) = 1 / (m·s² + b·s)
```

Questa è un **sistema del 2° ordine con un polo nell'origine** — intuitivamente: applicare una forza a un oggetto libero produce posizione (doppia integrazione).

---

## Concetto 2: Controllore PID

Il PID (Proportional-Integral-Derivative) è il controllore più usato nell'industria (>90% dei loop di controllo industriali).

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de/dt

dove e(t) = setpoint - misura = errore di quota
```

Nel dominio di Laplace:
```
C(s) = Kp + Ki/s + Kd·s
```

### Effetto di ogni termine

| Termine | Effetto | Problema |
|---------|---------|---------|
| **Kp** (proporzionale) | Velocità di risposta | Errore residuo a regime |
| **Ki** (integrale) | Elimina errore a regime | Può causare oscillazioni (windup) |
| **Kd** (derivativo) | Smorzamento, previsione | Amplifica il rumore |

### Anti-windup

Se l'uscita del PID satura (u > u_max), il termine integrale continua ad accumularsi — poi ci vuole molto tempo per "scaricarsi". Soluzione: **clamping**:

```python
if u_pid > U_MAX:
    u = U_MAX
    e_int -= e * dt   # annulla l'ultimo contributo: "freezing" dell'integrale
```

---

## Concetto 3: Metriche di prestazione

Dalla risposta a gradino (step response) si misurano:

```
Setpoint ──────────────────────────────────── 5.0 m
             ___
            /   \__________overshoot
           /               _________
          /               /
_________/               /
         |   |   |       |
         t0  t10 t90    t_settle

Rise time    = t90 - t10     (tempo per andare dal 10% al 90%)
Overshoot    = (max - SP) / SP × 100%
Settling time = tempo per rimanere entro ±2% del setpoint
SS error     = |setpoint - valore finale|
```

### Risultati del confronto

| Config | Overshoot | Rise time | Settling | SS Error |
|--------|-----------|-----------|---------|----------|
| Aggressiva (Kp=8) | ~18% | 0.6 s | 6.2 s | ~0 |
| Conservativa (Kp=2) | ~0% | 2.8 s | 4.1 s | ~0 |
| **Ottimale (Kp=4)** | **~4%** | **1.2 s** | **3.8 s** | **~0** |

---

## Concetto 4: Analisi in frequenza (Diagramma di Bode)

Il **diagramma di Bode** mostra come il sistema risponde a segnali sinusoidali a diverse frequenze.

Per l'anello aperto L(jω) = C(jω) · G(jω):

```
Modulo [dB] = 20·log₁₀(|L(jω)|)
Fase   [°]  = ∠L(jω)
```

### Margine di fase (Phase Margin)

Il margine di fase è quanto "margine" abbiamo prima che il sistema diventi instabile:
```
PM = 180° + fase(L) valutata a |L| = 0 dB
```
- PM > 45° → sistema stabile con buon margine
- PM > 60° → risposta smorzata (desiderabile)
- PM < 0°  → sistema instabile!

Per la configurazione Ottimale: **PM ≈ 52°** — stabile con buon margine.

---

## Concetto 5: Ciclo Model → Code (MBD workflow)

Il vero valore del Model Based Design è il **ciclo chiuso**:

```
1. Modello matematico del plant (G(s))
        ↓
2. Progetto del controllore in Octave/Simulink
        ↓
3. Simulazione e verifica metriche
        ↓
4. Export automatico dei guadagni → pid_gains.h
        ↓
5. Il firmware C include pid_gains.h

#include "pid_gains.h"
float Kp = PID_ALT_KP;   // 4.0000
float Ki = PID_ALT_KI;   // 0.8000
float Kd = PID_ALT_KD;   // 2.0000
```

In Simulink questo processo è completamente automatico (code generation da Embedded Coder). Il nostro script Octave/Python riproduce lo stesso workflow manualmente.

---

## Prossimo passo: Fase 6

**Hardware Design (KiCad) + Verification & Validation (V&V)**:
- Schema elettrico del sensor board (MCU + IMU + GPS + barometro)
- PCB layout
- Test unitari per firmware C (scheduler, sensori, telemetria)
- Test unitari per Ground Station Python (parser)
- Report di copertura del codice
