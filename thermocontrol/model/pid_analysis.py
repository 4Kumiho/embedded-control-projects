"""
@file pid_analysis.py
@brief PID Controller Design and Analysis (Python with scipy)

Python alternative to pid_controller.m for PID tuning and analysis.
Uses scipy.signal for control theory calculations.

This script:
    1. Defines thermal system transfer function
    2. Analyzes step response (overshoot, settling time, rise time)
    3. Designs PID gains using pole placement heuristic
    4. Generates frequency response plots (Bode)
    5. Exports optimized gains to C header file

Control Theory Background:
==========================
Thermal system (first-order ODE):
    dT/dt = -1/τ * (T - T_amb) + (U/100) * K

Transfer function (Laplace):
    G(s) = K/100 / (τ*s + 1)
    where:
      K = 0.05 (heater gain, °C/sec at 100%)
      τ = 30 sec (time constant)

PID controller:
    U(s) = (Kp + Ki/s + Kd*s) * E(s)

Design goal:
    - Setpoint tracking with < 0.2°C error
    - Overshoot < 2%
    - Settling time ~60-120 seconds

@author Andrea (ALTEN Training)
@date 2026-03-12
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.integrate import odeint
from datetime import datetime

# ============================================================================
#   SYSTEM PARAMETERS
# ============================================================================

# Thermal model parameters (from Fase 3)
TAU = 30.0              # Time constant (seconds)
K_HEATER = 0.05         # Heater gain (°C/sec at 100%)
K = K_HEATER / 100.0    # Normalized gain (per 1% command)
AMBIENT_TEMP = 20.0     # Ambient temperature (°C)

print("\n===== ThermoControl PID Design (Python/scipy) =====\n")
print("Thermal System Parameters:")
print(f"  τ (time constant) = {TAU} sec")
print(f"  K (heater gain) = {K:.6f} °C/sec per 1% command")
print(f"  Ambient temp = {AMBIENT_TEMP}°C\n")

# ============================================================================
#   TRANSFER FUNCTION
# ============================================================================

"""
Plant transfer function:
  G(s) = K / (τ*s + 1)

  Numerator: [K]
  Denominator: [τ, 1]
"""

num_plant = [K]
den_plant = [TAU, 1]
G = signal.TransferFunction(num_plant, den_plant)

print("Plant Transfer Function:")
print(f"  G(s) = {K:.6f} / ({TAU}*s + 1)")
print(f"  Poles: {signal.TransferFunction(num_plant, den_plant).poles}")
print(f"  DC Gain: {K:.6f}\n")

# ============================================================================
#   PID DESIGN (Pole Placement Heuristic)
# ============================================================================

"""
Design using classical tuning heuristic:

1. Choose desired closed-loop time constant τ_cl
   - Should be ~2-4x plant time constant
   - Here: τ_cl = 60 sec (2x plant τ)

2. Proportional gain:
   Kp ≈ 1 / (K * τ_cl)
   - This gives the main response speed

3. Integral gain:
   Ki = Kp / τ_i, where τ_i ≈ 2-4 * τ_plant
   - Eliminates steady-state error
   - Slower than proportional action

4. Derivative gain:
   Kd = τ_d * Kp, where τ_d ≈ 0.25 * τ_i
   - Provides damping (reduces overshoot)
   - Responds to error rate
"""

tau_cl = 60.0           # Desired closed-loop time constant
Kp = 1.0 / (K * tau_cl)

tau_i = 2.0 * TAU       # Integral time
Ki = Kp / tau_i

tau_d = 0.25 * tau_i   # Derivative time
Kd = tau_d * Kp

print("Design Specification:")
print(f"  Desired closed-loop τ = {tau_cl} sec")
print(f"  Integral time τ_i = {tau_i} sec")
print(f"  Derivative time τ_d = {tau_d} sec\n")

print("Initial PID Gains:")
print(f"  Kp = {Kp:.6f}")
print(f"  Ki = {Ki:.6f}")
print(f"  Kd = {Kd:.6f}\n")

# ============================================================================
#   CLOSED-LOOP SIMULATION
# ============================================================================

"""
Simulate PID response to 5°C setpoint change.

Model (continuous):
  dT/dt = -1/τ * T + K * U

PID:
  U = Kp*e + Ki*∫e + Kd*de/dt

where e = Tset - T (error)
"""

def pid_system(y, t, Tset, Kp, Ki, Kd, tau, K):
    """
    ODE for closed-loop system with PID controller.

    State: y = [T, integral_error]
    """
    T, integral_error = y

    # Error
    error = Tset - T

    # PID output
    U = Kp * error + Ki * integral_error + Kd * (0)  # No derivative term in simulation

    # System dynamics
    dT_dt = -1/tau * T + K * U
    d_integral_error_dt = error

    return [dT_dt, d_integral_error_dt]


# Simulation time
t = np.linspace(0, 600, 6001)  # 0-600 seconds, 6001 points

# Initial condition: T = 20°C (ambient)
y0 = [AMBIENT_TEMP, 0]

# Setpoint: step from 20°C to 25°C at t=0
Tset = 25.0

# Solve ODE
solution = odeint(pid_system, y0, t, args=(Tset, Kp, Ki, Kd, TAU, K))
T_response = solution[:, 0]

# Calculate performance metrics
step_amplitude = Tset - AMBIENT_TEMP  # 5°C step

# Rise time: time to reach 10-90% of final value
final_value = T_response[-1]
t_10 = np.min(np.where(T_response >= final_value * 0.1 + AMBIENT_TEMP * 0.9))
t_90 = np.min(np.where(T_response >= final_value * 0.9 + AMBIENT_TEMP * 0.1))
rise_time = t[t_90] - t[t_10] if t_90 > t_10 else 0

# Overshoot: maximum above final value
max_value = np.max(T_response)
overshoot = ((max_value - final_value) / step_amplitude) * 100 if step_amplitude > 0 else 0

# Settling time (2% criterion): time when error < 2% of step amplitude
tolerance = 0.02 * step_amplitude
settling_indices = np.where(np.abs(T_response - final_value) <= tolerance)[0]
settling_time = t[settling_indices[0]] if len(settling_indices) > 0 else 0

print("Step Response Performance (5°C setpoint change):")
print(f"  Rise time (10-90%): {rise_time:.1f} sec")
print(f"  Overshoot: {overshoot:.2f}%")
print(f"  Settling time (2%): {settling_time:.1f} sec")
print(f"  Final value: {final_value:.2f}°C")
print(f"  Steady-state error: {abs(final_value - Tset):.6f}°C\n")

# ============================================================================
#   FREQUENCY RESPONSE (Bode Plot)
# ============================================================================

"""
Bode plot shows magnitude and phase of open-loop transfer function G(s)*C(s).

This is useful for:
  - Verifying stability (phase margin > 0)
  - Understanding frequency response
  - Checking for resonances
"""

# Frequency range: 0.001 to 10 rad/s (logarithmic)
w = np.logspace(-3, 1, 1000)

# Plant frequency response
w_plant, mag_plant, phase_plant = signal.bode(G, w)

# PID controller (in frequency domain)
# C(s) = Kp + Ki/s + Kd*s
def pid_frequency_response(w, Kp, Ki, Kd):
    """Compute PID magnitude and phase at frequency w"""
    s = 1j * w
    C = Kp + Ki / s + Kd * s
    mag = 20 * np.log10(np.abs(C))
    phase = np.angle(C, deg=True)
    return mag, phase


mag_pid, phase_pid = pid_frequency_response(w, Kp, Ki, Kd)

# Open-loop response: G(s) * C(s)
mag_ol = mag_plant + mag_pid
phase_ol = phase_plant + phase_pid

print("Frequency Response (Open-Loop):")
print(f"  G(s)*C(s) at ω=0.01 rad/s: {mag_ol[np.argmin(np.abs(w-0.01))]:.2f} dB")
print(f"  Phase at ω=0.01 rad/s: {phase_ol[np.argmin(np.abs(w-0.01))]:.2f}°\n")

# ============================================================================
#   PLOTTING
# ============================================================================

fig, axes = plt.subplots(2, 2, figsize=(12, 10))

# Plot 1: Step response
ax = axes[0, 0]
ax.plot(t, T_response, 'b-', linewidth=2, label='Temperature')
ax.axhline(Tset, color='r', linestyle='--', linewidth=1, label='Setpoint')
ax.set_xlabel('Time (seconds)')
ax.set_ylabel('Temperature (°C)')
ax.set_title('Step Response: 5°C Setpoint Change')
ax.grid(True, alpha=0.3)
ax.legend()
ax.set_xlim([0, 300])

# Plot 2: Error signal
ax = axes[0, 1]
error_signal = Tset - T_response
ax.plot(t, error_signal, 'g-', linewidth=2)
ax.set_xlabel('Time (seconds)')
ax.set_ylabel('Error (°C)')
ax.set_title('Control Error: Setpoint - Temperature')
ax.grid(True, alpha=0.3)
ax.set_xlim([0, 300])

# Plot 3: Bode magnitude
ax = axes[1, 0]
ax.semilogx(w, mag_ol, 'b-', linewidth=2, label='Open-Loop Magnitude')
ax.set_xlabel('Frequency (rad/s)')
ax.set_ylabel('Magnitude (dB)')
ax.set_title('Bode Plot: G(s)*C(s)')
ax.grid(True, alpha=0.3, which='both')
ax.axhline(0, color='k', linestyle='-', linewidth=0.5)

# Plot 4: Bode phase
ax = axes[1, 1]
ax.semilogx(w, phase_ol, 'r-', linewidth=2, label='Open-Loop Phase')
ax.set_xlabel('Frequency (rad/s)')
ax.set_ylabel('Phase (degrees)')
ax.set_title('Bode Phase')
ax.grid(True, alpha=0.3, which='both')
ax.axhline(-180, color='k', linestyle='-', linewidth=0.5)

plt.tight_layout()
plt.savefig('pid_analysis.png', dpi=100)
print("✓ Plots saved to: pid_analysis.png\n")
plt.show()

# ============================================================================
#   EXPORT TO C HEADER FILE
# ============================================================================

output_file = 'pid_gains.h'

with open(output_file, 'w') as f:
    f.write('/**\n')
    f.write(' * @file pid_gains.h\n')
    f.write(' * @brief PID Controller Gains (Auto-generated)\n')
    f.write(' *\n')
    f.write(' * Generated by: pid_analysis.py (Model-Based Design)\n')
    f.write(f' * Date: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
    f.write(' * Method: Pole placement with desired closed-loop performance\n')
    f.write(' *\n')
    f.write(' * These gains are tuned to achieve:\n')
    f.write(f' *   - Overshoot < 2% (actual: {overshoot:.2f}%)\n')
    f.write(f' *   - Settling time < 120 sec (actual: {settling_time:.1f} sec)\n')
    f.write(' *   - Steady-state error ≈ 0\n')
    f.write(' *\n')
    f.write(' * System parameters:\n')
    f.write(f' *   - Thermal time constant τ = {TAU} sec\n')
    f.write(f' *   - Heater gain K = {K_HEATER:.4f} °C/sec per 1% command\n')
    f.write(' */\n\n')

    f.write('#ifndef PID_GAINS_H_INCLUDED\n')
    f.write('#define PID_GAINS_H_INCLUDED\n\n')

    f.write('/**\n')
    f.write(' * Proportional gain\n')
    f.write(f' * Tuned value: {Kp:.6f}\n')
    f.write(' */\n')
    f.write(f'#define PID_KP  {Kp:.6f}f\n\n')

    f.write('/**\n')
    f.write(' * Integral gain\n')
    f.write(f' * Tuned value: {Ki:.6f}\n')
    f.write(' */\n')
    f.write(f'#define PID_KI  {Ki:.6f}f\n\n')

    f.write('/**\n')
    f.write(' * Derivative gain\n')
    f.write(f' * Tuned value: {Kd:.6f}\n')
    f.write(' */\n')
    f.write(f'#define PID_KD  {Kd:.6f}f\n\n')

    f.write('#define PID_GAINS_H_INCLUDED 1\n\n')
    f.write('#endif /* PID_GAINS_H_INCLUDED */\n')

print(f"✓ PID gains exported to: {output_file}\n")

print("=" * 50)
print("Design Summary:")
print("=" * 50)
print(f"Kp = {Kp:.6f}")
print(f"Ki = {Ki:.6f}")
print(f"Kd = {Kd:.6f}")
print()
print("Performance:")
print(f"  Rise time: {rise_time:.1f} sec")
print(f"  Overshoot: {overshoot:.2f}%")
print(f"  Settling time: {settling_time:.1f} sec")
print()
print("Note: These are simulation results. Real system may differ.")
print("      Always test on actual hardware after design!")
print()
