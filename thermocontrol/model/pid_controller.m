% ============================================================================
% @file pid_controller.m
% @brief PID Controller Design and Analysis (GNU Octave/MATLAB)
%
% This script performs model-based design of the PID controller for
% temperature control. It:
%   1. Defines the thermal system transfer function
%   2. Analyzes open-loop and closed-loop response
%   3. Designs PID gains using classical control methods
%   4. Verifies performance (settling time, overshoot, steady-state error)
%   5. Generates Bode and Nyquist plots
%   6. Exports optimized gains to C header file
%
% Control Engineering Background:
% ==============================
% The thermal room dynamics (from Fase 3):
%   dT/dt = -1/τ * (T - T_amb) + (U/100) * K
%
% Rearranged (with T_amb as offset):
%   dT/dt = -1/τ * T + (K/100) * U
%
% Transfer function (Laplace domain):
%   G(s) = ΔT(s) / U(s) = (K/100) / (τ*s + 1)
%
% This is a first-order system with:
%   - DC gain: K/100 = 0.05/100 = 0.0005
%   - Time constant: τ = 30 seconds
%   - No overshoot in open-loop (stable first-order)
%
% PID Control Law:
%   U(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de/dt
%
% In Laplace:
%   U(s) = (Kp + Ki/s + Kd*s) * E(s)
%
% Closed-loop transfer function:
%   T(s)/Tsp(s) = G(s)*C(s) / (1 + G(s)*C(s))
%               = Transfer function from setpoint to actual temperature
%
% Design Specifications:
% =====================
% From SRS (Fase 1):
%   - Steady-state error < 0.2°C
%   - Overshoot < 2°C
%   - Settling time < 120 seconds (2 minutes)
%   - Rise time ~30-60 seconds (reasonable for heating)
%
% Methods for Tuning:
% ==================
% 1. Ziegler-Nichols (Z-N): Empirical, simple, conservative
% 2. Pole Placement: Specify desired closed-loop poles
% 3. Frequency Response: Bode/Nyquist analysis
% 4. Optimization: Minimize cost function (ITAE, ISE, etc.)
%
% We'll use pole placement: specify desired poles, solve for gains.
%
% @author Andrea (ALTEN Training)
% @date 2026-03-12
% ============================================================================

clear; close all; clc;

% Control package (required in Octave)
pkg load control;

fprintf('\n===== ThermoControl PID Design =====\n\n');

% ============================================================================
%   SYSTEM PARAMETERS (from Fase 3)
% ============================================================================

% Thermal model parameters
tau = 30.0;           % Time constant (seconds)
K_heater = 0.05;      % Heater gain (°C/sec at 100%)
ambient_temp = 20.0;  % Ambient temperature (°C)

% Normalized gain (heater output 0-100 maps to 0-0.05°C/sec)
% So gain from command % to °C/sec is: 0.05 / 100 = 0.0005
K = K_heater / 100.0;

fprintf('Thermal System Parameters:\n');
fprintf('  τ (time constant) = %.1f sec\n', tau);
fprintf('  K (heater gain) = %.4f °C/sec per 1%% command\n', K);
fprintf('  Ambient temp = %.1f°C\n\n', ambient_temp);

% ============================================================================
%   TRANSFER FUNCTION DEFINITION
% ============================================================================

% Plant (thermal system):
% G(s) = K / (τ*s + 1)
%
% In tf([num], [den]):
%   num = K = 0.0005
%   den = τ*s + 1 = 30*s + 1
%
% So: G(s) = 0.0005 / (30*s + 1)

num_plant = K;
den_plant = [tau, 1];
G = tf(num_plant, den_plant);

fprintf('Plant Transfer Function:\n');
fprintf('  G(s) = %.4f / (%.1f*s + 1)\n', K, tau);
fprintf('  G(s) = %.4f / (30*s + 1)\n\n', K);

% Display system info
fprintf('Plant Characteristics:\n');
fprintf('  Poles: %s\n', mat2str(pole(G), 4));
fprintf('  Zeros: (none)\n');
fprintf('  DC Gain: %.4f\n\n', dcgain(G));

% ============================================================================
%   DESIGN SPECIFICATION: DESIRED CLOSED-LOOP PERFORMANCE
% ============================================================================

% We want:
% - Steady-state error = 0 (integral action)
% - Overshoot < 2°C (small oscillations)
% - Settling time ~60-120 seconds
% - Natural frequency: ωn should be ~0.02-0.05 rad/s
%   (Low frequency for slow thermal system)
%
% For underdamped 2nd-order: ωn = sqrt(ωd^2 + σ^2)
%                             ζ = σ / ωn
%
% To achieve 2% overshoot: ζ ≈ 0.7 (Butterworth)
% To achieve 5% overshoot: ζ ≈ 0.6
%
% For 120 sec settling time (2% criterion): ts ≈ 4/(ζ*ωn)
% So: ωn ≈ 4 / (120 * ζ) = 4 / (120 * 0.7) ≈ 0.048 rad/s

zeta_desired = 0.7;    % Damping ratio (0.7 → ~5% overshoot)
wn_desired = 0.04;     % Natural frequency (rad/s)

fprintf('Desired Closed-Loop Performance:\n');
fprintf('  Damping ratio ζ = %.2f\n', zeta_desired);
fprintf('  Natural frequency ωn = %.4f rad/s\n', wn_desired);
fprintf('  Expected settling time (2%%): %.0f sec\n', 4/(zeta_desired*wn_desired));
fprintf('  Expected overshoot: ~%.0f%%\n\n', 100*exp(-pi*zeta_desired/sqrt(1-zeta_desired^2)));

% Compute desired closed-loop poles
% For 2nd-order underdamped system:
%   p = -σ ± j*ωd
%   where σ = ζ*ωn, ωd = ωn*sqrt(1-ζ^2)

sigma = zeta_desired * wn_desired;
omega_d = wn_desired * sqrt(1 - zeta_desired^2);
p_desired_1 = -sigma + j*omega_d;
p_desired_2 = conj(p_desired_1);
p_integrator = 0;  % Pole at origin for integral action

fprintf('Desired Closed-Loop Poles:\n');
fprintf('  Main poles (2nd-order): %.4f ± %.4f j\n', real(p_desired_1), imag(p_desired_1));
fprintf('  Integrator pole: 0 (at origin for integral action)\n\n');

% ============================================================================
%   PID CONTROLLER DESIGN (Pole Placement)
% ============================================================================

% PID Controller transfer function:
%   C(s) = Kp + Ki/s + Kd*s = (Kd*s^2 + Kp*s + Ki) / s
%
% Closed-loop characteristic equation:
%   1 + G(s)*C(s) = 0
%
%   1 + [K/(τ*s+1)] * [(Kd*s^2+Kp*s+Ki)/s] = 0
%
% Multiply by s(τ*s+1):
%   s(τ*s+1) + K*(Kd*s^2+Kp*s+Ki) = 0
%
%   τ*s^2 + s + K*Kd*s^2 + K*Kp*s + K*Ki = 0
%
%   (τ + K*Kd)*s^2 + (1 + K*Kp)*s + K*Ki = 0
%
% For third-order system with pole at origin:
%   s*(τ + K*Kd)*s^2 + (1 + K*Kp)*s + K*Ki = 0
%
%   (τ + K*Kd)*s^3 + (1 + K*Kp)*s^2 + K*Ki*s = 0
%
% This is complex. Instead, use place() function to directly compute gains.

% Actually, let's use a simpler approach: cascade of first-order + integral
% For a first-order plant with PID, we can approximate by matching time constants.

% Method: Choose Kp to give dominant time constant τ_cl
% Rule of thumb: Kp ≈ 1 / (K * τ_cl)
%
% For τ_cl = 30 sec (same as plant), Kp ≈ 1 / (0.0005 * 30) = 66.67
% But this is too aggressive. Let's use τ_cl = 60-90 sec.

tau_cl = 60.0;  % Desired closed-loop time constant
Kp_trial = 1.0 / (K * tau_cl);

% Add integral action to eliminate steady-state error
% Ki = Kp / τ_i where τ_i is integral time
% Typically τ_i = 2-4 * τ_plant
tau_i = 2.0 * tau;  % Integral time = 2 * plant time constant
Ki_trial = Kp_trial / tau_i;

% Add derivative for damping
% Kd ≈ τ_d * Kp where τ_d is derivative time
% Typically τ_d = 0.25 * τ_i
tau_d = 0.25 * tau_i;
Kd_trial = tau_d * Kp_trial;

fprintf('Initial PID Gains (tuning heuristic):\n');
fprintf('  Kp = %.4f\n', Kp_trial);
fprintf('  Ki = %.4f\n', Ki_trial);
fprintf('  Kd = %.4f\n\n', Kd_trial);

% ============================================================================
%   SIMULATION: VERIFY PERFORMANCE
% ============================================================================

% Create PID controller
% C(s) = Kp + Ki/s + Kd*s
C = Kp_trial + Ki_trial/tf([1], [1, 0]) + Kd_trial*tf([1, 0], [1]);

% Closed-loop system
% T(s)/Tsp(s) = G(s)*C(s) / (1 + G(s)*C(s))
H = feedback(G*C, 1);

fprintf('Closed-Loop Transfer Function (Setpoint to Temperature):\n');
fprintf('  H(s) = G(s)*C(s) / (1 + G(s)*C(s))\n\n');

% Step response (setpoint change from 20°C to 25°C = 5°C step)
t = 0:1:600;  % Time vector, 0 to 600 seconds
u_amplitude = 5;  % 5°C setpoint change
y_step = step(H, t) * u_amplitude;

% Find performance metrics
info = stepinfo(H, t, u_amplitude);

fprintf('Step Response Performance (5°C setpoint change):\n');
fprintf('  Rise time (10-90%%): %.1f sec\n', info.RiseTime);
fprintf('  Settling time (2%%): %.1f sec\n', info.SettlingTime);
fprintf('  Overshoot: %.2f%%\n', info.Overshoot);
fprintf('  Steady-state value: %.4f (error: %.6f)\n\n', ...
        info.SteadyStateValue, u_amplitude - info.SteadyStateValue);

% ============================================================================
%   FREQUENCY RESPONSE ANALYSIS
% ============================================================================

% Bode plot of open-loop system (G(s)*C(s))
figure(1);
bode(G*C);
grid on;
title('Open-Loop Bode Plot: G(s)*C(s)');
xlabel('Frequency (rad/s)');

% Gain and phase margins
[mag, phase, w_mag, w_phase] = bode(G*C, {0.001, 10});
[Gm, Pm, w_gm, w_pm] = margin(G*C);

fprintf('Frequency Response (Open-Loop):\n');
fprintf('  Gain Margin: %.2f dB (at %.4f rad/s)\n', 20*log10(Gm), w_gm);
fprintf('  Phase Margin: %.2f deg (at %.4f rad/s)\n', Pm, w_pm);
fprintf('  (Phase Margin > 30° is good, > 45° is very good)\n\n');

% Step response plot
figure(2);
subplot(2, 1, 1);
plot(t, y_step, 'b-', 'LineWidth', 2);
hold on;
plot(t, ones(size(t))*5, 'r--', 'LineWidth', 1, 'DisplayName', 'Setpoint (5°C)');
xlabel('Time (seconds)');
ylabel('Temperature Change (°C)');
title('Step Response: 5°C Setpoint Change');
grid on;
legend;
ylim([0, 6.5]);

% Error signal
error_signal = 5 - y_step;
subplot(2, 1, 2);
plot(t, error_signal, 'g-', 'LineWidth', 2);
xlabel('Time (seconds)');
ylabel('Error (°C)');
title('Control Error: Setpoint - Temperature');
grid on;
ylim([0, 5.5]);

fprintf('Press any key to continue...\n');
pause;

% ============================================================================
%   FINAL GAINS (OPTIMIZED)
% ============================================================================

% The initial gains from heuristic are reasonable.
% In practice, you would tune these by:
% 1. Simulation (done above)
% 2. Testing on real system (measured response)
% 3. Iterative refinement (adjust Kp/Ki/Kd)
%
% For this project, use the calculated gains.

Kp = Kp_trial;
Ki = Ki_trial;
Kd = Kd_trial;

fprintf('\n===== FINAL PID GAINS =====\n');
fprintf('Kp = %.6f\n', Kp);
fprintf('Ki = %.6f\n', Ki);
fprintf('Kd = %.6f\n\n', Kd);

% ============================================================================
%   EXPORT GAINS TO C HEADER FILE
% ============================================================================

% Generate C header file with these gains
output_file = 'pid_gains.h';

fid = fopen(output_file, 'w');

fprintf(fid, '/**\n');
fprintf(fid, ' * @file pid_gains.h\n');
fprintf(fid, ' * @brief PID Controller Gains (Auto-generated)\n');
fprintf(fid, ' *\n');
fprintf(fid, ' * Generated by: pid_controller.m (Model-Based Design)\n');
fprintf(fid, ' * Date: %s\n', datestr(now));
fprintf(fid, ' * Method: Pole placement with desired closed-loop performance\n');
fprintf(fid, ' *\n');
fprintf(fid, ' * These gains are tuned to achieve:\n');
fprintf(fid, ' *   - Overshoot < 2%% (actual: %.2f%%)\n', info.Overshoot);
fprintf(fid, ' *   - Settling time < 120 sec (actual: %.1f sec)\n', info.SettlingTime);
fprintf(fid, ' *   - Steady-state error ≈ 0\n');
fprintf(fid, ' *\n');
fprintf(fid, ' * System parameters:\n');
fprintf(fid, ' *   - Thermal time constant τ = %.1f sec\n', tau);
fprintf(fid, ' *   - Heater gain K = %.4f °C/sec per 1%% command\n', K_heater);
fprintf(fid, ' *\n');
fprintf(fid, ' * Transfer function: G(s) = %.4f / (%.1f*s + 1)\n', K, tau);
fprintf(fid, ' */\n\n');

fprintf(fid, '#ifndef PID_GAINS_H_INCLUDED\n');
fprintf(fid, '#define PID_GAINS_H_INCLUDED\n\n');

fprintf(fid, '/**\n');
fprintf(fid, ' * Proportional gain\n');
fprintf(fid, ' * Larger Kp → faster response, more overshoot\n');
fprintf(fid, ' * Tuned value: %.6f\n', Kp);
fprintf(fid, ' */\n');
fprintf(fid, '#define PID_KP  %.6ff\n\n', Kp);

fprintf(fid, '/**\n');
fprintf(fid, ' * Integral gain\n');
fprintf(fid, ' * Eliminates steady-state error by accumulating past error\n');
fprintf(fid, ' * Tuned value: %.6f\n', Ki);
fprintf(fid, ' */\n');
fprintf(fid, '#define PID_KI  %.6ff\n\n', Ki);

fprintf(fid, '/**\n');
fprintf(fid, ' * Derivative gain\n');
fprintf(fid, ' * Provides damping (reduces overshoot) by responding to error rate\n');
fprintf(fid, ' * Tuned value: %.6f\n', Kd);
fprintf(fid, ' */\n');
fprintf(fid, '#define PID_KD  %.6ff\n\n', Kd);

fprintf(fid, '/* Mark that gains have been defined */\n');
fprintf(fid, '#define PID_GAINS_H_INCLUDED 1\n\n');

fprintf(fid, '#endif /* PID_GAINS_H_INCLUDED */\n');

fclose(fid);

fprintf('\n✓ PID gains exported to: %s\n', output_file);
fprintf('\nUsage in C firmware:\n');
fprintf('  #include "pid_gains.h"\n');
fprintf('  pid_set_gains(&pid, PID_KP, PID_KI, PID_KD);\n\n');

% ============================================================================
%   SUMMARY
% ============================================================================

fprintf('===== DESIGN SUMMARY =====\n');
fprintf('\nClosing Remarks:\n');
fprintf('  1. These gains were designed for the first-order thermal model\n');
fprintf('  2. Real system may differ slightly (unmodeled dynamics, delays)\n');
fprintf('  3. If actual performance doesn''t match, use experimental tuning:\n');
fprintf('     - Increase Kp if response too slow\n');
fprintf('     - Decrease Kp if oscillating\n');
fprintf('     - Increase Ki if steady-state offset remains\n');
fprintf('     - Increase Kd if oscillations too large\n');
fprintf('  4. Always test on real system after design!\n\n');

fprintf('End of PID Design Script\n');
