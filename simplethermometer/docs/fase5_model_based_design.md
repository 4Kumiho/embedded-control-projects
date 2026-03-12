# Fase 5 — Model-Based Design (Sensor Characterization)

**Durata:** ~30 minuti
**Competenze coperte:** Model-Based Design, Transfer Function Analysis, Simulation

---

## Panoramica

Fase 5 caratterizza il modello del sensore usando teoria di controllo:
1. Definire transfer function del sensore
2. Analizzare step response
3. Estrarre tempo di risposta caratteristico
4. Importare parametri in firmware

---

## Competenza ALTEN Coperta

✅ **Model-Based Design** — Transfer function, step response, simulation

---

## Modello del Sensore

### Equazione Differenziale

Il sensore simula un sistema primo ordine:

```
dT/dt = -1/τ * (T - T_amb) + K * U

Dove:
  T = temperature (output)
  T_amb = ambient temperature = 20°C
  τ = time constant = 0.5 seconds (very fast filter!)
  K = gain = 1.0
  U = input signal
```

### Transfer Function (Laplace)

```
G(s) = K / (τ*s + 1)
     = 1.0 / (0.5*s + 1)

Pole:
  s = -1/τ = -2.0 rad/s (stable, fast decay)

DC Gain:
  G(0) = K = 1.0
```

### Time Constant

```
τ = 0.5 seconds

Tempo di salita (10% → 90%):
  tr ≈ 2.2*τ = 1.1 secondi

Tempo di assestamento (2%):
  ts ≈ 4*τ = 2.0 secondi
```

**Interpretazione:**
- Il filtro risponde MOLTO velocemente (0.5s)
- Dopo 2 secondi, è al 95% del valore finale
- Cambierà il valore in circa 1 secondo

---

## Step Response Analysis

### Scenario: Input Step 5°C

```
Condizioni iniziali:
  T(0) = 20°C (ambient)
  U = 5.0 (step input)

Analiticamente:
  T(t) = 20 + 5*(1 - e^(-t/0.5))

Risultati:
  t=0.5s → T = 20 + 5*(1 - e^(-1)) ≈ 23.2°C (63%)
  t=1.0s → T = 20 + 5*(1 - e^(-2)) ≈ 24.3°C (86%)
  t=2.0s → T = 20 + 5*(1 - e^(-4)) ≈ 24.9°C (98%)
  t=∞   → T = 25.0°C (100%)
```

### Plot Step Response

```
Temperature (°C)
     |
   25├─────────────────────────ₓ Final value
     |                    ╱
   24├            ╱──────
     |       ╱────
   23├   ╱────
     | ╱
   20├─────────────────────────
     |
     └┴─────┴─────┴─────┴─────→ Time (seconds)
     0     1     2     3     4

Caratteristiche:
  - Rise time: ~1.1 seconds
  - Settling time: ~2.0 seconds
  - Overshoot: 0% (first-order, sempre sottodampato)
  - Steady-state error: 0°C
```

---

## Frequency Response (Bode Plot)

### Magnitude

```
|G(jω)| = K / √(1 + (ωτ)²)

In dB:
  20*log₁₀(|G(jω)|) = 20*log₁₀(K) - 20*log₁₀(√(1 + (ωτ)²))

Punti caratteristici:
  ω = 0: Gain = 0 dB
  ω = 1/τ = 2 rad/s: Gain = -3 dB (cutoff frequency)
  ω → ∞: Gain → -40 dB/decade (first-order rolloff)
```

### Phase

```
∠G(jω) = -arctan(ωτ)

Punti caratteristici:
  ω = 0: Phase = 0°
  ω = 1/τ = 2 rad/s: Phase = -45°
  ω → ∞: Phase → -90°
```

### Bode Plot

```
Magnitude (dB)
         0├─────ₓ
         -3├     ╲
          -10├     ╲╲
          -20├      ╲╲
          -30├       ╲╲
             └┴┴┴┴┴┴─────→ log₁₀(ω)
            0.1  1  10

Cutoff frequency:
  ωc = 1/τ = 2 rad/s
  Attenuation at ωc: -3 dB


Phase (degrees)
           0├───────ₓ
         -45├        ╲
         -90├         ╲───
             └┴┴┴┴┴┴─────→ log₁₀(ω)
            0.1  1  10
```

---

## Parametri nel Firmware

### sensor.c (Parametri Attuali)

```c
/* Time constant (seconds) */
float alpha = 0.1f;

/* Filter: new = α*current + (1-α)*previous */
float filtered = alpha * temp_new + (1.0f - alpha) * prev_temp;
```

**Equivalenza:**
```
α = 0.1 corrisponde a τ ≈ 0.05 secondi (MOLTO veloce)
α = 0.5 corrisponde a τ ≈ 0.5 secondi (veloce)
α = 0.9 corrisponde a τ ≈ 5.0 secondi (lento)
```

**Formula esatta:**
```
τ = Ts / (2*α - 1)

Dove Ts = 0.5s (periodo di sampling)

α = 0.1 → τ = 0.5 / (0.2 - 1) ≈ 0.056s
α = 0.5 → τ = 0.5 / (1.0 - 1) = ∞ (no filtering)
```

---

## Implicazioni Pratiche

### Variazione nel Tempo Reale

```
Ogni 500ms, il sensore legge:
- 10% del nuovo valore
- 90% del valore precedente

Risultato:
- Cambiamenti lenti e realistici
- Nessun noise visibile nel plot GUI
- Stabilità garantita
```

### Noise Suppression

```
Input rumore: ±0.5°C
Output dopo filtro: ~±0.05°C (10x attenuazione)

A frequenza noise (random):
  ω >> ωc → Attenuazione di -40 dB/decade

Esempio:
  Noise frequency = 10 Hz
  Attenuazione ≈ -60 dB = 1/1000 dell'ampiezza
```

---

## Modificare i Parametri

Se il sensore è troppo veloce/lento, cambiare `α` in sensor.c:

```c
/* Più veloce (meno filtraggio) */
float alpha = 0.2f;  // Risponde più rapidamente

/* Più lento (più filtraggio) */
float alpha = 0.05f;  // Filtra più aggressivamente
```

**Trade-off:**
```
α grande (0.5):
  ✓ Risponde velocemente
  ✗ Passa più rumore

α piccolo (0.05):
  ✓ Filtra bene
  ✗ Risponde lentamente
```

---

## Competenze Demonstrate

1. **Transfer Functions** — Rappresentazione sistemi LTI
2. **Step Response** — Analisi nel dominio del tempo
3. **Bode Plot** — Analisi nel dominio della frequenza
4. **System Identification** — Misurare/determinare τ
5. **Digital Filters** — Implementare IIR filter
6. **Trade-offs** — Equilibrio tra velocità e filtraggio

---

## Prossimo: Completamento

Progetto completato!

Competenze coperte:
- ✅ Requirements Engineering (SRS)
- ✅ SW Embedded C (Firmware)
- ✅ SW OOP Python (GUI)
- ✅ Testing (Unit Tests)
- ✅ Model-Based Design

---

## Referenze

**Transfer Functions:**
- https://en.wikipedia.org/wiki/Transfer_function
- "Control Systems Engineering" by Nise

**Bode Plot:**
- https://en.wikipedia.org/wiki/Bode_plot
- MIT OCW: Signals and Systems

**Digital Filters:**
- https://en.wikipedia.org/wiki/Infinite_impulse_response
- "The Scientist and Engineer's Guide to DSP"

---
