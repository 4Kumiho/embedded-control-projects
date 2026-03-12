# Sensor Board — Descrizione Schematico

**Progetto:** AeroSim Sensor Board v1.0
**Tool:** KiCad 7.x (file .kicad_sch)
**Data:** 2026-03-11

---

## Architettura del sistema

```
                    ┌─────────────────────────────────────────┐
                    │           Sensor Board v1.0             │
                    │                                         │
  USB/UART ────────►│  STM32F411CEU6 (MCU)                   │
  (debug/fw)        │  ┌─────────────┐                        │
                    │  │  ARM         │◄──── SWD (J1)         │
                    │  │  Cortex-M4  │                        │
  3.3V LDO ────────►│  │  100MHz     │                        │
  (AMS1117)         │  │  512KB Flash│                        │
                    │  │  128KB RAM  │                        │
                    │  └──────┬──────┘                        │
                    │         │                               │
              ┌─────┼─────────┼─────────────────────┐        │
              │     │  I2C1   │  SPI1    UART1       │        │
              │     │  PB8/9  │  PA5-7   PA9/10      │        │
              ▼     │         │          │            │        │
           MPU-6050 │      BMP390    ESP32-MINI   NEO-M9N     │
           (IMU)    │    (Baro/SPI)  (WiFi/UART) (GPS/UART)  │
                    │                                         │
                    └─────────────────────────────────────────┘
```

---

## Componenti principali

### U1 — STM32F411CEU6 (MCU)
- **Package:** UFQFPN48
- **Core:** ARM Cortex-M4, FPU integrata
- **Clock:** fino a 100 MHz
- **Flash:** 512 KB | **RAM:** 128 KB
- **Perifiche usate:** I2C1, SPI1, UART1, UART2, TIM2
- **Alimentazione:** 3.3V (decoupling: 100nF + 10µF su ogni pin VDD)
- **Motivo scelta:** FPU integrata per i calcoli IMU, abbondanza di UART, basso costo

### U2 — MPU-6050 (IMU)
- **Package:** QFN24
- **Interfaccia:** I2C (PB8=SCL, PB9=SDA) @ 400 kHz (Fast Mode)
- **Indirizzo I2C:** 0x68 (pin AD0 a GND)
- **Alimentazione:** 3.3V
- **Sensori interni:** accelerometro ±16g + giroscopio ±2000°/s
- **Interrupt:** INT → PA0 (EXTI0) per data ready
- **Decoupling:** 100nF tra VCC e GND, il più vicino possibile al package
- **Motivo scelta:** diffusissimo, driver open-source disponibili, buone prestazioni per uso didattico

### U3 — BMP390 (Barometro + Temperatura)
- **Package:** LGA8
- **Interfaccia:** SPI (PA5=SCK, PA6=MISO, PA7=MOSI, PA4=CS_BARO)
- **Range:** 300-1250 hPa | **Risoluzione:** 0.016 Pa
- **Alimentazione:** 3.3V (VDDIO separato)
- **Motivo scelta:** alta risoluzione per altimetria di precisione

### U4 — u-blox NEO-M9N (GPS)
- **Package:** 12.1×16mm SMD
- **Interfaccia:** UART (PA9=TX, PA10=RX) @ 115200 baud
- **Protocollo:** UBX binario (configurato via UART all'avvio)
- **Antenna:** connettore U.FL per antenna esterna
- **Alimentazione:** 3.3V (con batteria di backup CR1220 per hot start)
- **Fix:** fino a 25 Hz update rate, CEP < 2.5m
- **Motivo scelta:** alta frequenza di update, supporto multi-costellazione

### U5 — ESP32-MINI-1 (Modulo WiFi/BT per telemetria)
- **Package:** modulo SMD 13.2×16.6mm
- **Interfaccia:** UART2 (PA2=TX2, PA3=RX2) @ 115200 baud
- **Protocollo:** AT commands o firmware custom (socket TCP)
- **Alimentazione:** 3.3V (attenzione: picchi fino a 500mA durante TX)
- **Motivo scelta:** TCP/IP stack built-in, non serve stack di rete sull'MCU

### Regolatore — AMS1117-3.3
- **Input:** 5V (USB o batteria LiPo 1S=4.2V)
- **Output:** 3.3V @ 1A
- **Decoupling:** 10µF electrolittico + 100nF ceramico su IN e OUT

---

## Connettori

| Ref | Tipo | Segnali |
|-----|------|---------|
| J1 | 10-pin SWD | SWDIO, SWDCLK, GND, 3.3V, NRST |
| J2 | USB Micro-B | 5V, D+, D-, GND (per alimentazione e debug) |
| J3 | 2-pin JST-PH | Batteria LiPo 1S |
| J4 | U.FL | Antenna GPS esterna |
| J5 | 4-pin JST-SH | UART espansione |

---

## Schema elettrico (ASCII art — sezione MCU + IMU)

```
         VDD_MCU (3.3V)
              │
             ─┴─
            100nF    10µF
             ─┬─      │
              │        │
         ┌────┴────────┴────┐
         │   STM32F411CEU6  │
         │                  │
         │ PB8 (SCL) ───────┼────────────────── SCL ──┐
         │ PB9 (SDA) ───────┼────────────────── SDA   │
         │                  │                     │   │  MPU-6050
         │ PA0 (INT) ◄──────┼──────────────── INT─┤   │  ┌────────┐
         │                  │                     └───┼──►│ VCC    │
         │ PA4 (CS_BARO)────┼──────────────────────  │   │ SCL    │
         │ PA5 (SCK) ───────┼──────────────────────  └───►│ SDA    │
         │ PA6 (MISO)◄──────┼──────────────────────      │ INT    │
         │ PA7 (MOSI)───────┼──────────────────────      │ AD0─GND│
         │                  │                            │ GND    │
         │ PA9  (TX1)───────┼─► GPS RX                  └────────┘
         │ PA10 (RX1)◄──────┼── GPS TX
         │                  │
         │ PA2  (TX2)───────┼─► ESP32 RX
         │ PA3  (RX2)◄──────┼── ESP32 TX
         └──────────────────┘

Pull-up I2C:  4.7kΩ tra SCL→3.3V e SDA→3.3V
```

---

## Bill of Materials (BOM)

Vedi: [BOM.csv](BOM.csv)

---

## Design Rules (DRC KiCad)

| Regola | Valore |
|--------|--------|
| Min trace width | 0.15 mm |
| Min clearance | 0.15 mm |
| Min via drill | 0.3 mm |
| Min via annular ring | 0.1 mm |
| Copper layers | 4 (Signal / GND / PWR / Signal) |
| Board size | 50×40 mm |

### Stack-up PCB a 4 layer:
```
Layer 1 (Top):    segnali + componenti SMD
Layer 2:          GND plane (continuo, per EMI/EMC)
Layer 3:          PWR plane (3.3V e 5V)
Layer 4 (Bottom): segnali + componenti SMD
```

Il piano di GND continuo al layer 2 minimizza le interferenze tra
moduli RF (ESP32, GPS) e segnali analogici (IMU).
