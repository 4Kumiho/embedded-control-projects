% =========================================================================
% pid_controller.m  —  Modello PID per controllo di quota del drone
%
% Competenza ALTEN: Model Based Design (Matlab/Simulink)
%
% Questo script implementa e analizza un controllore PID per la
% stabilizzazione della quota di un quadricottero.
%
% Contenuto:
%   1. Modello del sistema (plant) — dinamica verticale del drone
%   2. Controllore PID con anti-windup
%   3. Simulazione risposta a gradino
%   4. Analisi delle prestazioni (overshoot, settling time, rise time)
%   5. Confronto tra diverse sintonizzazioni
%   6. Esportazione guadagni come header C
%
% Avvio: octave pid_controller.m
%        oppure aprire in Matlab/Octave IDE
% =========================================================================

clear; clc; close all;
printf('=== AeroSim PID Controller Design ===\n\n');

% =========================================================================
% 1. PARAMETRI DEL SISTEMA (PLANT)
% =========================================================================
%
% Dinamica verticale linearizzata attorno all'hovering:
%
%   m * z'' = u - b * z'
%
%   dove:
%     z   = quota [m]
%     u   = deviazione di spinta rispetto all'hovering [N]
%     m   = massa del drone [kg]
%     b   = coefficiente di smorzamento aerodinamico [N*s/m]
%
% Funzione di trasferimento:
%
%          1
%   G(s) = ─────────
%          m*s² + b*s
%
% Questa è un'approssimazione lineare valida per piccole deviazioni
% attorno all'equilibrio di hovering.

m = 0.8;      % massa drone [kg]       — da DroneParams::default_params()
b = 0.25;     % smorzamento [N*s/m]   — da drag_coeff (semplificato)
g = 9.81;     % gravità [m/s²]

% Spinta di hovering (thrust = peso):  T_hover = m*g
T_hover = m * g;
printf('Massa drone:      %.2f kg\n', m);
printf('Spinta hovering:  %.2f N\n', T_hover);
printf('Smorzamento:      %.2f N*s/m\n\n', b);

% =========================================================================
% 2. PARAMETRI SIMULAZIONE
% =========================================================================

dt    = 0.001;   % passo di integrazione [s] — identico al firmware (1ms)
t_end = 12.0;    % durata simulazione [s]
t     = 0 : dt : t_end;
N     = length(t);

% Setpoint: il drone deve salire da 0 m a 5 m
setpoint = 5.0;   % [m]

% Limiti anti-windup (deviazione di spinta ammessa)
u_max =  T_hover * 0.9;   % +90% della spinta hovering
u_min = -T_hover * 0.6;   % -60% della spinta hovering

% =========================================================================
% 3. FUNZIONE DI SIMULAZIONE PID
% =========================================================================

function [z_h, zdot_h, u_h, e_h] = simulate_pid(Kp, Ki, Kd, ...
                                                   m, b, dt, N, ...
                                                   setpoint, u_min, u_max)
  % Inizializzazione stato
  z      = 0.0;   % quota iniziale [m]
  zdot   = 0.0;   % velocità verticale iniziale [m/s]
  e_int  = 0.0;   % integrale errore
  e_prev = 0.0;   % errore al passo precedente

  % Pre-allocazione (più efficiente che append in loop)
  z_h    = zeros(1, N);
  zdot_h = zeros(1, N);
  u_h    = zeros(1, N);
  e_h    = zeros(1, N);

  for i = 1:N
    % ---- Errore di posizione ----
    e = setpoint - z;

    % ---- Termine integrale (con anti-windup back-calculation) ----
    e_int = e_int + e * dt;

    % ---- Termine derivativo (derivata dell'errore) ----
    %      Nota: nella pratica si usa spesso la derivata della misura
    %      (non dell'errore) per evitare il "derivative kick" a gradino.
    e_dot = (e - e_prev) / dt;
    e_prev = e;

    % ---- Uscita PID ----
    u_pid = Kp * e + Ki * e_int + Kd * e_dot;

    % ---- Saturazione + anti-windup (clamping method) ----
    if u_pid > u_max
      u = u_max;
      e_int = e_int - e * dt;   % annulla l'ultimo contributo integrale
    elseif u_pid < u_min
      u = u_min;
      e_int = e_int - e * dt;
    else
      u = u_pid;
    end

    % ---- Dinamica plant: m*z'' = u - b*z' (Eulero esplicito) ----
    %
    % In produzione si userebbe RK4 (come nel physics engine),
    % ma per l'analisi del controllore Eulero è sufficiente
    % con dt = 1ms.
    zddot = (u - b * zdot) / m;
    zdot  = zdot + zddot * dt;
    z     = z    + zdot   * dt;

    % ---- Salvataggio ----
    z_h(i)    = z;
    zdot_h(i) = zdot;
    u_h(i)    = u;
    e_h(i)    = e;
  end
end

% =========================================================================
% 4. ANALISI DELLE PRESTAZIONI
% =========================================================================

function metrics = analyze_response(t, z_h, setpoint, dt)
  metrics = struct();
  N = length(z_h);

  % --- Overshoot ---
  % Percentuale di superamento del valore finale
  z_max = max(z_h);
  if z_max > setpoint
    metrics.overshoot_pct = (z_max - setpoint) / setpoint * 100;
  else
    metrics.overshoot_pct = 0;
  end

  % --- Rise time: tempo per andare dal 10% al 90% del setpoint ---
  t10 = NaN; t90 = NaN;
  for i = 1:N
    if isnan(t10) && z_h(i) >= 0.10 * setpoint
      t10 = t(i);
    end
    if isnan(t90) && z_h(i) >= 0.90 * setpoint
      t90 = t(i);
      break;
    end
  end
  metrics.rise_time = t90 - t10;

  % --- Settling time: tempo per rimanere entro ±2% del setpoint ---
  tolerance = 0.02 * setpoint;
  metrics.settling_time = t(end);   % default: non si assesta mai
  for i = N:-1:1
    if abs(z_h(i) - setpoint) > tolerance
      metrics.settling_time = t(min(i + 1, N));
      break;
    end
  end

  % --- Errore a regime (steady-state error) ---
  metrics.ss_error = abs(setpoint - z_h(end));
end

% =========================================================================
% 5. SINTONIZZAZIONE — confronto tra diverse configurazioni
% =========================================================================
%
% Guadagni PID da confrontare:
%   Config A: aggressiva (alto Kp) → risposta rapida ma oscillante
%   Config B: conservativa (basso Kp) → lenta ma stabile
%   Config C: ottimale (tuning manuale bilanciato)

configs = {
%   Nome            Kp    Ki    Kd
  'Aggressiva',    8.0,  1.5,  1.0;
  'Conservativa',  2.0,  0.3,  3.0;
  'Ottimale',      4.0,  0.8,  2.0;
};

colors  = {'r', 'b', 'g'};
results = {};

printf('%-16s  %10s  %10s  %10s  %10s\n', ...
       'Config', 'Overshoot%', 'Rise[s]', 'Settle[s]', 'SSError[m]');
printf('%s\n', repmat('-', 1, 62));

for c = 1:size(configs, 1)
  name = configs{c,1};
  Kp   = configs{c,2};
  Ki   = configs{c,3};
  Kd   = configs{c,4};

  [z_h, ~, u_h, e_h] = simulate_pid(Kp, Ki, Kd, m, b, dt, N, ...
                                      setpoint, u_min, u_max);
  metrics = analyze_response(t, z_h, setpoint, dt);
  results{c} = struct('z', z_h, 'u', u_h, 'e', e_h, ...
                       'name', name, 'Kp', Kp, 'Ki', Ki, 'Kd', Kd, ...
                       'metrics', metrics);

  printf('%-16s  %10.1f  %10.3f  %10.3f  %10.4f\n', ...
         name, metrics.overshoot_pct, metrics.rise_time, ...
         metrics.settling_time, metrics.ss_error);
end

% =========================================================================
% 6. GRAFICI
% =========================================================================

figure(1, 'name', 'Risposta a gradino — Confronto PID');
set(gcf, 'Position', [100, 100, 900, 700]);

% ---- Subplot 1: Quota ----
subplot(3, 1, 1);
hold on;
plot(t, setpoint * ones(1, N), 'k--', 'LineWidth', 1.5, ...
     'DisplayName', 'Setpoint');
for c = 1:length(results)
  plot(t, results{c}.z, colors{c}, 'LineWidth', 1.5, ...
       'DisplayName', results{c}.name);
end
hold off;
ylabel('Quota [m]');
title('Risposta a gradino — Controllo quota drone');
legend('Location', 'southeast');
grid on;
ylim([-0.5, setpoint * 1.4]);

% ---- Subplot 2: Uscita PID (u) ----
subplot(3, 1, 2);
hold on;
for c = 1:length(results)
  plot(t, results{c}.u, colors{c}, 'LineWidth', 1.2, ...
       'DisplayName', results{c}.name);
end
yline(u_max, 'k:', 'u_{max}');
yline(u_min, 'k:', 'u_{min}');
hold off;
ylabel('Deviazione spinta [N]');
title('Uscita controllore PID');
legend('Location', 'northeast');
grid on;

% ---- Subplot 3: Errore ----
subplot(3, 1, 3);
hold on;
for c = 1:length(results)
  plot(t, results{c}.e, colors{c}, 'LineWidth', 1.2, ...
       'DisplayName', results{c}.name);
end
yline(0, 'k--');
yline( 0.02*setpoint, 'k:', '+2%');
yline(-0.02*setpoint, 'k:', '-2%');
hold off;
xlabel('Tempo [s]');
ylabel('Errore [m]');
title('Errore (setpoint - quota)');
legend('Location', 'northeast');
grid on;

% =========================================================================
% 7. GRAFICO RISPOSTA IN FREQUENZA (Bode semplificato)
%    Calcola il guadagno in anello aperto in funzione della frequenza
% =========================================================================

figure(2, 'name', 'Bode — Anello aperto (Config Ottimale)');
set(gcf, 'Position', [200, 200, 700, 500]);

% Configurazione ottimale
opt = results{3};
Kp = opt.Kp; Ki = opt.Ki; Kd = opt.Kd;

% Frequenze di analisi
omega = logspace(-2, 3, 500);   % rad/s

% Funzione di trasferimento in anello aperto: L(jw) = C(jw) * G(jw)
%   C(jw) = Kp + Ki/(jw) + Kd*jw
%   G(jw) = 1 / (m*(jw)^2 + b*(jw))

jw  = 1j * omega;
C   = Kp + Ki ./ jw + Kd .* jw;
G   = 1 ./ (m .* jw.^2 + b .* jw);
L   = C .* G;

subplot(2, 1, 1);
semilogx(omega, 20*log10(abs(L)), 'b', 'LineWidth', 1.5);
yline(0, 'k--', '0 dB');
xlabel('Frequenza [rad/s]');
ylabel('Modulo [dB]');
title('Bode — Modulo anello aperto L(jω)');
grid on;

subplot(2, 1, 2);
phase_deg = angle(L) * 180 / pi;
semilogx(omega, phase_deg, 'r', 'LineWidth', 1.5);
yline(-180, 'k--', '-180°');
xlabel('Frequenza [rad/s]');
ylabel('Fase [°]');
title('Bode — Fase anello aperto L(jω)');
grid on;

% Calcola margini di stabilità
[~, idx_gc] = min(abs(abs(L) - 1));   % crossover di guadagno (|L|=1)
phase_margin = 180 + phase_deg(idx_gc);
printf('\nConfig Ottimale:\n');
printf('  Kp=%.1f  Ki=%.1f  Kd=%.1f\n', Kp, Ki, Kd);
printf('  Margine di fase: %.1f°  (>45° = stabile)\n', phase_margin);

% =========================================================================
% 8. ESPORTAZIONE GUADAGNI COME HEADER C
% =========================================================================
%
% Il firmware C importerà questo file per usare i guadagni calcolati
% dal modello, chiudendo il ciclo Model → Code.

opt_idx = 3;   % usiamo la config ottimale
Kp_out  = results{opt_idx}.Kp;
Ki_out  = results{opt_idx}.Ki;
Kd_out  = results{opt_idx}.Kd;

header_path = 'pid_gains.h';
fid = fopen(header_path, 'w');
fprintf(fid, '/**\n');
fprintf(fid, ' * @file pid_gains.h\n');
fprintf(fid, ' * @brief Guadagni PID generati da model/pid_controller.m\n');
fprintf(fid, ' *\n');
fprintf(fid, ' * NON MODIFICARE MANUALMENTE — file generato automaticamente.\n');
fprintf(fid, ' * Modificare pid_controller.m e rieseguire il modello.\n');
fprintf(fid, ' *\n');
fprintf(fid, ' * Configurazione: %s\n', results{opt_idx}.name);
fprintf(fid, ' * Plant: m=%.2f kg, b=%.2f N*s/m\n', m, b);
fprintf(fid, ' * Overshoot: %.1f%%  Settling: %.3f s\n', ...
        results{opt_idx}.metrics.overshoot_pct, ...
        results{opt_idx}.metrics.settling_time);
fprintf(fid, ' */\n\n');
fprintf(fid, '#ifndef PID_GAINS_H\n');
fprintf(fid, '#define PID_GAINS_H\n\n');
fprintf(fid, '/* Guadagni PID — controllo quota */\n');
fprintf(fid, '#define PID_ALT_KP  %.4ff   /* proporzionale */\n', Kp_out);
fprintf(fid, '#define PID_ALT_KI  %.4ff   /* integrale     */\n', Ki_out);
fprintf(fid, '#define PID_ALT_KD  %.4ff   /* derivativo    */\n\n', Kd_out);
fprintf(fid, '/* Limiti anti-windup */\n');
fprintf(fid, '#define PID_ALT_U_MAX  %.4ff   /* N */\n',  u_max);
fprintf(fid, '#define PID_ALT_U_MIN  %.4ff   /* N */\n\n', u_min);
fprintf(fid, '#endif /* PID_GAINS_H */\n');
fclose(fid);

printf('\nGuadagni esportati in: %s\n', header_path);
printf('  Kp = %.4f\n', Kp_out);
printf('  Ki = %.4f\n', Ki_out);
printf('  Kd = %.4f\n', Kd_out);
printf('\nScript completato.\n');
