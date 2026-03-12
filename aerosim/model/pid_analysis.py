"""
pid_analysis.py — Alternativa Python allo script Octave.

Stessa analisi PID ma usando numpy + matplotlib + scipy.
Utile per chi non ha Octave installato.

Dipendenze: pip install numpy matplotlib scipy
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from dataclasses import dataclass
from pathlib import Path

# ================================================================== #
#  Parametri del sistema (plant)                                       #
# ================================================================== #

M = 0.8      # massa drone [kg]
B = 0.25     # smorzamento aerodinamico [N*s/m]
G = 9.81     # gravità [m/s²]

T_HOVER = M * G   # spinta hovering [N]
U_MAX   =  T_HOVER * 0.9
U_MIN   = -T_HOVER * 0.6

# Simulazione
DT      = 0.001   # passo integrazione [s] — come firmware
T_END   = 12.0    # durata [s]
SETPOINT = 5.0    # quota target [m]

# ================================================================== #
#  Struttura risultati                                                 #
# ================================================================== #

@dataclass
class SimResult:
    name:    str
    Kp:      float
    Ki:      float
    Kd:      float
    t:       np.ndarray
    z:       np.ndarray
    zdot:    np.ndarray
    u:       np.ndarray
    error:   np.ndarray
    # Metriche
    overshoot_pct:  float = 0.0
    rise_time:      float = 0.0
    settling_time:  float = 0.0
    ss_error:       float = 0.0

# ================================================================== #
#  Simulatore PID                                                      #
# ================================================================== #

def simulate_pid(Kp: float, Ki: float, Kd: float,
                 name: str = "PID") -> SimResult:
    """
    Simula il sistema drone in quota con controllore PID.

    Plant (linearizzato attorno all'hovering):
        m * z'' = u - b * z'
        G(s) = 1 / (m*s² + b*s)

    Integrazione: Eulero esplicito (dt=1ms, sufficiente per l'analisi).
    """
    N  = int(T_END / DT) + 1
    t  = np.linspace(0, T_END, N)

    z      = 0.0   # quota [m]
    zdot   = 0.0   # velocità verticale [m/s]
    e_int  = 0.0   # integrale errore
    e_prev = 0.0   # errore precedente

    z_h    = np.zeros(N)
    zdot_h = np.zeros(N)
    u_h    = np.zeros(N)
    e_h    = np.zeros(N)

    for i in range(N):
        e = SETPOINT - z

        # Termine integrale
        e_int += e * DT

        # Termine derivativo
        e_dot  = (e - e_prev) / DT
        e_prev = e

        # Uscita PID
        u_pid = Kp * e + Ki * e_int + Kd * e_dot

        # Anti-windup (clamping)
        if u_pid > U_MAX:
            u = U_MAX
            e_int -= e * DT     # annulla l'ultimo contributo integrale
        elif u_pid < U_MIN:
            u = U_MIN
            e_int -= e * DT
        else:
            u = u_pid

        # Plant: m*z'' = u - b*z'
        zddot = (u - B * zdot) / M
        zdot  += zddot * DT
        z     += zdot  * DT

        z_h[i]    = z
        zdot_h[i] = zdot
        u_h[i]    = u
        e_h[i]    = e

    res = SimResult(name=name, Kp=Kp, Ki=Ki, Kd=Kd,
                    t=t, z=z_h, zdot=zdot_h, u=u_h, error=e_h)
    _compute_metrics(res, t, z_h)
    return res


def _compute_metrics(res: SimResult, t: np.ndarray, z: np.ndarray) -> None:
    """Calcola le metriche di prestazione dalla risposta."""
    N = len(z)

    # Overshoot
    z_max = np.max(z)
    res.overshoot_pct = max(0.0, (z_max - SETPOINT) / SETPOINT * 100)

    # Rise time (10% → 90%)
    t10 = t[np.argmax(z >= 0.10 * SETPOINT)]
    t90 = t[np.argmax(z >= 0.90 * SETPOINT)]
    res.rise_time = float(t90 - t10) if t90 > t10 else float('inf')

    # Settling time (±2%)
    tol = 0.02 * SETPOINT
    in_band = np.abs(z - SETPOINT) < tol
    # Cerca l'ultimo istante fuori banda
    out_idx = np.where(~in_band)[0]
    res.settling_time = float(t[out_idx[-1] + 1]) if out_idx.size > 0 else 0.0

    # Steady-state error
    res.ss_error = float(np.abs(SETPOINT - z[-1]))

# ================================================================== #
#  Analisi in frequenza (Bode manuale)                                 #
# ================================================================== #

def compute_open_loop_bode(Kp: float, Ki: float, Kd: float):
    """
    Calcola modulo e fase dell'anello aperto L(jω) = C(jω)·G(jω).

    C(jω) = Kp + Ki/(jω) + Kd·jω     (PID)
    G(jω) = 1 / (M·(jω)² + B·jω)     (plant)
    """
    omega = np.logspace(-2, 3, 500)
    jw    = 1j * omega
    C     = Kp + Ki / jw + Kd * jw
    G_tf  = 1.0 / (M * jw**2 + B * jw)
    L     = C * G_tf

    mag_db    = 20 * np.log10(np.abs(L))
    phase_deg = np.angle(L) * 180 / np.pi

    # Margine di fase: fase a |L|=0 dB
    gc_idx     = np.argmin(np.abs(np.abs(L) - 1.0))
    phase_margin = 180 + phase_deg[gc_idx]

    return omega, mag_db, phase_deg, phase_margin

# ================================================================== #
#  Visualizzazione                                                     #
# ================================================================== #

def plot_responses(results: list[SimResult]) -> None:
    colors = ['#e74c3c', '#3498db', '#2ecc71']
    fig = plt.figure(figsize=(12, 8), facecolor='#1e1e1e')
    fig.suptitle('Analisi PID — Controllo Quota Drone', color='white',
                 fontsize=14, fontweight='bold')

    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.3)

    ax_z = fig.add_subplot(gs[0, 0])
    ax_u = fig.add_subplot(gs[1, 0])
    ax_e = fig.add_subplot(gs[2, 0])
    ax_bode_mag = fig.add_subplot(gs[0, 1])
    ax_bode_ph  = fig.add_subplot(gs[1, 1])
    ax_metrics  = fig.add_subplot(gs[2, 1])

    def style_ax(ax, title, ylabel, xlabel=''):
        ax.set_facecolor('#2d2d2d')
        ax.set_title(title, color='white', fontsize=9, pad=4)
        ax.set_ylabel(ylabel, color='#aaa', fontsize=8)
        if xlabel:
            ax.set_xlabel(xlabel, color='#aaa', fontsize=8)
        ax.tick_params(colors='#aaa', labelsize=7)
        for s in ax.spines.values(): s.set_color('#555')
        ax.grid(True, color='#444', linewidth=0.5)

    # --- Quota ---
    style_ax(ax_z, 'Risposta a gradino — Quota', 'Quota [m]')
    ax_z.axhline(SETPOINT, color='white', ls='--', lw=1, label='Setpoint')
    ax_z.axhspan(SETPOINT*0.98, SETPOINT*1.02,
                 alpha=0.1, color='green', label='±2%')
    for res, col in zip(results, colors):
        ax_z.plot(res.t, res.z, color=col, lw=1.5, label=res.name)
    ax_z.legend(fontsize=7, labelcolor='white',
                facecolor='#333', edgecolor='#555')
    ax_z.set_ylim(-0.3, SETPOINT * 1.45)

    # --- Uscita PID ---
    style_ax(ax_u, 'Uscita controllore PID', 'u [N]')
    ax_u.axhline(U_MAX, color='#aaa', ls=':', lw=1)
    ax_u.axhline(U_MIN, color='#aaa', ls=':', lw=1)
    for res, col in zip(results, colors):
        ax_u.plot(res.t, res.u, color=col, lw=1.2, label=res.name)

    # --- Errore ---
    style_ax(ax_e, 'Errore', 'e [m]', 'Tempo [s]')
    ax_e.axhline(0,  color='white', ls='--', lw=1)
    ax_e.axhline( 0.02*SETPOINT, color='green', ls=':', lw=1)
    ax_e.axhline(-0.02*SETPOINT, color='green', ls=':', lw=1)
    for res, col in zip(results, colors):
        ax_e.plot(res.t, res.error, color=col, lw=1.2)

    # --- Bode (config ottimale) ---
    opt = results[2]
    omega, mag, phase, pm = compute_open_loop_bode(opt.Kp, opt.Ki, opt.Kd)

    style_ax(ax_bode_mag, 'Bode — Modulo (config Ottimale)', 'Modulo [dB]')
    ax_bode_mag.semilogx(omega, mag, color='#4fc3f7', lw=1.5)
    ax_bode_mag.axhline(0, color='white', ls='--', lw=1, label='0 dB')
    ax_bode_mag.legend(fontsize=7, labelcolor='white',
                       facecolor='#333', edgecolor='#555')

    style_ax(ax_bode_ph, 'Bode — Fase (config Ottimale)',
             'Fase [°]', 'Frequenza [rad/s]')
    ax_bode_ph.semilogx(omega, phase, color='#ef9a9a', lw=1.5)
    ax_bode_ph.axhline(-180, color='white', ls='--', lw=1, label='-180°')
    ax_bode_ph.set_ylim(-270, 0)
    ax_bode_ph.legend(fontsize=7, labelcolor='white',
                      facecolor='#333', edgecolor='#555')

    # --- Tabella metriche ---
    ax_metrics.set_facecolor('#2d2d2d')
    ax_metrics.axis('off')
    ax_metrics.set_title('Metriche di prestazione', color='white',
                         fontsize=9, pad=4)

    headers = ['Config', 'Overshoot', 'Rise[s]', 'Settle[s]', 'SSErr[m]']
    rows    = []
    for res in results:
        rows.append([
            res.name,
            f"{res.overshoot_pct:.1f}%",
            f"{res.rise_time:.3f}",
            f"{res.settling_time:.3f}",
            f"{res.ss_error:.4f}",
        ])

    tbl = ax_metrics.table(cellText=rows, colLabels=headers,
                           loc='center', cellLoc='center')
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_facecolor('#1e1e1e' if r == 0 else '#2d2d2d')
        cell.set_text_props(color='white')
        cell.set_edgecolor('#555')

    plt.savefig('pid_analysis.png', dpi=120, bbox_inches='tight',
                facecolor='#1e1e1e')
    print("Grafico salvato: pid_analysis.png")
    plt.show()

# ================================================================== #
#  Esportazione header C                                               #
# ================================================================== #

def export_gains_header(res: SimResult, path: str = "pid_gains.h") -> None:
    """
    Genera il file pid_gains.h con i guadagni PID ottimizzati.
    Questo header viene incluso direttamente dal firmware C.
    """
    content = f"""\
/**
 * @file pid_gains.h
 * @brief Guadagni PID generati da model/pid_analysis.py
 *
 * NON MODIFICARE MANUALMENTE — file generato automaticamente.
 * Modificare pid_analysis.py e rieseguire il modello.
 *
 * Configurazione: {res.name}
 * Plant: m={M:.2f} kg, b={B:.2f} N*s/m
 * Overshoot: {res.overshoot_pct:.1f}%  Settling: {res.settling_time:.3f} s
 */

#ifndef PID_GAINS_H
#define PID_GAINS_H

/* Guadagni PID — controllo quota */
#define PID_ALT_KP  {res.Kp:.4f}f   /* proporzionale */
#define PID_ALT_KI  {res.Ki:.4f}f   /* integrale     */
#define PID_ALT_KD  {res.Kd:.4f}f   /* derivativo    */

/* Limiti anti-windup */
#define PID_ALT_U_MAX   {U_MAX:.4f}f   /* N */
#define PID_ALT_U_MIN  {U_MIN:.4f}f   /* N */

#endif /* PID_GAINS_H */
"""
    Path(path).write_text(content)
    print(f"Header C esportato: {path}")

# ================================================================== #
#  Main                                                                #
# ================================================================== #

def main():
    print("=== AeroSim PID Analysis (Python) ===\n")
    print(f"Plant: m={M} kg, b={B} N*s/m, T_hover={T_HOVER:.2f} N\n")

    # Configurazioni da confrontare
    configs = [
        ("Aggressiva",   8.0, 1.5, 1.0),
        ("Conservativa", 2.0, 0.3, 3.0),
        ("Ottimale",     4.0, 0.8, 2.0),
    ]

    results = []
    print(f"{'Config':<16} {'Overshoot':>10} {'Rise[s]':>10} "
          f"{'Settle[s]':>10} {'SSErr[m]':>10}")
    print("-" * 62)

    for name, Kp, Ki, Kd in configs:
        res = simulate_pid(Kp, Ki, Kd, name)
        results.append(res)
        print(f"{res.name:<16} {res.overshoot_pct:>9.1f}% "
              f"{res.rise_time:>10.3f} {res.settling_time:>10.3f} "
              f"{res.ss_error:>10.4f}")

    # Margine di fase config ottimale
    opt = results[2]
    _, _, _, pm = compute_open_loop_bode(opt.Kp, opt.Ki, opt.Kd)
    print(f"\nConfig Ottimale — margine di fase: {pm:.1f}°  (>45° = stabile)")

    # Esporta header
    export_gains_header(opt)

    # Plot
    plot_responses(results)


if __name__ == "__main__":
    main()
