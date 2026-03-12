# Fase 3 — SW OOP (C++) e Physics Engine

## Cosa abbiamo fatto

Nella Fase 3 abbiamo implementato il **motore fisico** in C++, coprendo la competenza ALTEN:
- **SW OOP**: progettazione orientata agli oggetti, classi, incapsulamento
- **FW**: interfaccia C/C++ per integrazione con il firmware

---

## File creati

| File | Ruolo |
|------|-------|
| `physics/Vec3.h` | Vettore 3D header-only con operatori algebrici |
| `physics/Environment.h/cpp` | Gravità, densità aria, modello vento Dryden |
| `physics/Drone.h/cpp` | Corpo rigido 6DOF + integratore RK4 |
| `physics/drone_c_api.h/cpp` | Bridge C/C++ con `extern "C"` |

---

## Concetto 1: Modello 6DOF (Six Degrees of Freedom)

Un corpo rigido nello spazio ha **6 gradi di libertà**:
- 3 traslazioni: posizione (x, y, z)
- 3 rotazioni: orientamento (roll φ, pitch θ, yaw ψ)

Il **vettore di stato** del drone ha 12 componenti:

```
stato = [ px, py, pz,        ← posizione nel frame mondo [m]
          vx, vy, vz,        ← velocità nel frame mondo [m/s]
          φ,  θ,  ψ,         ← angoli di Eulero [rad]
          p,  q,  r  ]       ← velocità angolari nel frame corpo [rad/s]
```

---

## Concetto 2: Equazioni di Newton-Euler

### Traslazione (seconda legge di Newton)
```
dp/dt = v                          (posizione cambia con la velocità)
dv/dt = F_totale / massa           (velocità cambia con la forza)
```

### Rotazione (equazioni di Eulero per corpo rigido)
```
dη/dt = W(η) · ω                  (angoli cambiano con le velocità angolari)
dω/dt = I⁻¹ · [M - ω × (I·ω)]   (omega cambia con i momenti)
```

dove:
- `η` = angoli di Eulero (roll, pitch, yaw)
- `ω` = velocità angolari nel frame corpo (p, q, r)
- `I` = tensore di inerzia (matrice 3x3, diagonale per drone simmetrico)
- `M` = momenti (coppie) applicati
- `ω × (I·ω)` = termine giroscopico (causa la precessione)

---

## Concetto 3: Matrice di rotazione (corpo → mondo)

Per applicare la spinta dei motori nel frame mondo bisogna ruotare il vettore dal frame corpo:
```
F_mondo = R · F_corpo
```

La matrice R (ZYX, convenzione aerospace) è:
```
        ⎡ cψcθ    cψsθsφ-sψcφ    cψsθcφ+sψsφ ⎤
R =     ⎢ sψcθ    sψsθsφ+cψcφ    sψsθcφ-cψsφ ⎥
        ⎣ -sθ        cθsφ             cθcφ    ⎦
```
dove c = cos, s = sin, φ=roll, θ=pitch, ψ=yaw.

---

## Concetto 4: Configurazione motori quadricottero

```
         FRONTE
    M0(CCW)  M1(CW)
        \   /
         \ /
          +
         / \
        /   \
    M3(CW)  M2(CCW)
         RETRO
```

Ogni motore genera:
- **Spinta Ti = kT · ωi²** (forza verso +Z corpo)
- **Coppia Qi = kD · ωi²** (coppia di reazione attorno Z)

I differenziali di spinta creano i momenti:

| Momento | Formula | Azione |
|---------|---------|--------|
| Roll (L) | `l·kT·(T1+T2 - T0-T3)` | Inclina a destra/sinistra |
| Pitch (M) | `l·kT·(T0+T1 - T2-T3)` | Inclina avanti/indietro |
| Yaw (N) | `kD/kT·(T1+T3 - T0-T2)` | Ruota in piano |

---

## Concetto 5: Runge-Kutta 4° ordine (RK4)

L'integrazione numerica risponde alla domanda: *dato lo stato ora, qual è lo stato tra dt secondi?*

**Eulero (semplice ma impreciso):**
```
y(t+dt) = y(t) + dt · f(y(t))
```
Errore: O(dt²) — diverge rapidamente a passi grandi.

**RK4 (standard industriale):**
```
k1 = f(y)
k2 = f(y + dt/2 · k1)
k3 = f(y + dt/2 · k2)
k4 = f(y + dt   · k3)

y(t+dt) = y(t) + dt/6 · (k1 + 2·k2 + 2·k3 + k4)
```
Errore: O(dt⁵) — circa 1000× più accurato di Eulero con lo stesso dt.

RK4 è usato in: X-Plane, JSBSim, ArduPilot SITL, simulatori satellite ESA.

```cpp
// Nel nostro codice
DroneState k1 = compute_derivative(m_state, m_motors);
DroneState k2 = compute_derivative(state_add(m_state, k1, dt*0.5), m_motors);
DroneState k3 = compute_derivative(state_add(m_state, k2, dt*0.5), m_motors);
DroneState k4 = compute_derivative(state_add(m_state, k3, dt),     m_motors);
// poi si combinano con i pesi RK4...
```

---

## Concetto 6: interfaccia C/C++ con extern "C"

Il firmware è in C, il physics engine è in C++. Il problema: **C++ usa il name mangling** — il compilatore rinomina le funzioni aggiungendo il tipo degli argomenti. Questo rompe la compatibilità con C.

Soluzione: `extern "C"` istruisce il compilatore C++ a **non** fare mangling:

```cpp
// drone_c_api.h
#ifdef __cplusplus
extern "C" {
#endif

void drone_init(float wind_x, float wind_y);   // simbolo C puro
void drone_step(uint32_t dt_ms);

#ifdef __cplusplus
}
#endif
```

Il firmware C include solo `drone_c_api.h` e non sa nulla di classi o oggetti.

---

## OOP: scelte progettuali

| Principio OOP | Dove lo applichiamo |
|---------------|---------------------|
| **Incapsulamento** | Stato interno (`m_state`, `m_motors`) privato in `Drone` |
| **Separazione responsabilità** | `Environment` gestisce forze esterne, `Drone` la dinamica |
| **Header-only** | `Vec3.h` è una classe leggera senza .cpp (zero overhead) |
| **Singleton** | `drone_c_api.cpp` usa puntatori globali statici (pattern embedded) |
| **const-correctness** | `state()` ritorna `const DroneState &`, metodi `const` dove possibile |

---

## Prossimo passo: Fase 4

Sviluppo della **Ground Station in Python**:
- GUI PyQt6 con indicatore di assetto artificiale
- Grafici real-time di quota e velocità
- Parser del protocollo binario (da `telemetry.bin`)
- Mappa 2D della posizione GPS
