/**
 * @file pid_gains.h
 * @brief Guadagni PID generati da model/pid_controller.m
 *
 * NON MODIFICARE MANUALMENTE — file generato automaticamente.
 * Modificare pid_controller.m (o pid_analysis.py) e rieseguire il modello.
 *
 * Configurazione: Ottimale
 * Plant: m=0.80 kg, b=0.25 N*s/m
 * Overshoot: 4.2%  Settling: 3.847 s
 */

#ifndef PID_GAINS_H
#define PID_GAINS_H

/* Guadagni PID — controllo quota */
#define PID_ALT_KP  4.0000f   /* proporzionale */
#define PID_ALT_KI  0.8000f   /* integrale     */
#define PID_ALT_KD  2.0000f   /* derivativo    */

/* Limiti anti-windup */
#define PID_ALT_U_MAX   7.0632f   /* N */
#define PID_ALT_U_MIN  -4.7088f   /* N */

#endif /* PID_GAINS_H */
