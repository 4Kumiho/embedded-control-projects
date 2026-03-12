# Fase 6 — HW Design + Verifica e Validazione (V&V)

## Cosa abbiamo fatto

Nella Fase 6 abbiamo completato il progetto coprendo le ultime due competenze ALTEN:
- **HW Design**: schematico, PCB, protocolli, BOM
- **V&V**: test unitari firmware (C) e GCS (Python), copertura, CI/CD

---

## File creati

| File | Ruolo |
|------|-------|
| `hardware/sensor_board/schematic_description.md` | Descrizione schematico con ASCII art |
| `hardware/sensor_board/BOM.csv` | Bill of Materials con codici Mouser |
| `tests/test_firmware/test_framework.h` | Test framework C lightweight |
| `tests/test_firmware/test_sensors.c` | 11 test per il modulo sensori |
| `tests/test_firmware/test_telemetry.c` | 13 test per il protocollo telemetria |
| `tests/test_firmware/test_scheduler.c` | 6 test per lo scheduler |
| `tests/test_firmware/test_runner.c` | Entry point test suite C |
| `tests/test_ground/test_parser.py` | 22 test pytest per il parser Python |

---

## Come eseguire i test

```bash
# Test firmware (C)
mkdir build && cd build
cmake ..
make test_firmware
ctest -V           # oppure: ./tests/test_firmware/test_firmware

# Test ground station (Python)
cd tests/test_ground
pip install pytest
pytest test_parser.py -v
```

---

## Concetto 1: V&V — Verifica vs Validazione

Due concetti distinti in ambito safety-critical (DO-178C, ISO 26262):

| | Verifica | Validazione |
|---|---------|-------------|
| Domanda | "Abbiamo costruito il sistema correttamente?" | "Abbiamo costruito il sistema giusto?" |
| Metodo | Test unitari, review del codice, analisi statica | Test di sistema, test di accettazione utente |
| Chi | Team di sviluppo | Cliente / certificatore |
| Quando | Durante lo sviluppo | Al termine |

Nel nostro progetto:
- **Verifica** = test unitari C e Python (abbiamo fatto la fase 6)
- **Validazione** = eseguire il sistema completo e controllare che il drone simulato si comporti come atteso

---

## Concetto 2: Test unitari C — il nostro test framework

Non usiamo Google Test (richiede C++) ma un framework C puro:

```c
// Definizione test
TEST(checksum_self_inverse) {
    uint8_t payload[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t csum = telemetry_checksum(payload, 4);

    uint8_t full[5];
    memcpy(full, payload, 4);
    full[4] = csum;

    // XOR(payload + checksum) deve essere sempre 0
    ASSERT_EQ(0x00, telemetry_checksum(full, 5));
}

// Esecuzione nel main
RUN_TEST(checksum_self_inverse);
```

### Macro ASSERT

```c
// Confronto interi
ASSERT_EQ(expected, actual)

// Float con tolleranza assoluta (indispensabile per floating point)
ASSERT_NEAR(expected, actual, tolerance)

// Condizione booleana
ASSERT_TRUE(condition)
```

### Perché ASSERT_NEAR per i float?

```c
// SBAGLIATO — confronto diretto float è unreliable!
assert(sensor.az == 9.81f);

// CORRETTO — tolleranza esplicita
ASSERT_NEAR(9.81, mean_az, 0.5);   // ±0.5 m/s² di tolleranza
```

I float hanno errori di rappresentazione (IEEE 754 non può rappresentare esattamente tutti i decimali). Confrontarli direttamente con `==` è quasi sempre un bug.

---

## Concetto 3: Test statistici per dati stocastici

I sensori generano dati con rumore gaussiano — non possiamo aspettarci un valore esatto. La soluzione: **testare la media su N campioni** (legge dei grandi numeri).

```c
#define N_SAMPLES 1000

sensors_init(42);      // seed fisso → risultati riproducibili
double sum_az = 0.0;
IMUData d;
for (int i = 0; i < N_SAMPLES; i++) {
    sensors_read_imu(&d);
    sum_az += d.az;
}
double mean_az = sum_az / N_SAMPLES;

// Con 1000 campioni, la media deve essere entro ±0.5 m/s² da 9.81
ASSERT_NEAR(9.81, mean_az, 0.5);
```

Con N=1000 e σ=0.05 m/s², l'errore standard della media è σ/√N = 0.05/31.6 ≈ 0.0016 m/s² — molto inferiore alla tolleranza di 0.5.

---

## Concetto 4: Pytest — test Python professionali

```python
class TestStreamHandling:

    def test_partial_then_complete(self):
        """Dati incompleti + completamento = 1 pacchetto."""
        full = build_imu_packet()
        parser = TelemetryParser()

        parser.feed(full[:10])              # solo metà pacchetto
        assert len(parser.parse_all()) == 0 # non ancora emesso

        parser.feed(full[10:])              # resto del pacchetto
        assert len(parser.parse_all()) == 1 # ora emesso
```

**Organizzazione**: test raggruppati in classi per modulo logico.
**Fixture**: `tmp_path` fornita da pytest crea directory temporanee automaticamente.
**Parametrizzazione**: si può usare `@pytest.mark.parametrize` per lo stesso test con input diversi.

---

## Concetto 5: HW Design — scelte progettuali

### Protocolli di comunicazione

| Sensore | Protocollo | Frequenza | Motivo |
|---------|-----------|-----------|--------|
| MPU-6050 (IMU) | I2C | 400 kHz | Standard per sensori lenti, solo 2 fili |
| BMP390 (Baro) | SPI | 10 MHz | Più veloce di I2C, utile per alta freq. |
| NEO-M9N (GPS) | UART | 115200 baud | GPS usa UART per semplicità |
| ESP32 (WiFi) | UART | 115200 baud | AT commands, no stack di rete sull'MCU |

### I2C vs SPI

```
I2C:
  ┌────┐  SCL  ┌───────┐  SCL  ┌───────┐
  │MCU │──────►│ IMU-1 │──────►│ IMU-2 │  (multi-device, 2 fili)
  │    │──────►│       │──────►│       │
  └────┘  SDA  └───────┘  SDA  └───────┘

SPI:
  ┌────┐  SCK/MOSI/MISO  ┌────────┐
  │MCU │────────────────►│ Baro-1 │  (un CS per device, 4 fili)
  │    │──CS1────────────►│        │
  │    │──CS2────────────►│ Baro-2 │
  └────┘                  └────────┘
```

### PCB 4-layer — perché?

Un PCB a 4 layer con piano di GND al layer 2 garantisce:
- **EMC**: riduce le emissioni RF del modulo WiFi (ESP32)
- **Impedenza controllata**: le trace RF devono avere impedenza di 50Ω
- **Stabilità alimentazione**: piano PWR dedicato riduce il rumore

---

## Riepilogo copertura test

| Modulo | Test C | Test Python | Copertura stimata |
|--------|--------|-------------|-------------------|
| sensors.c | 11 | — | ~85% |
| telemetry.c | 13 | — | ~90% |
| scheduler.c | 6 | — | ~75% |
| parser.py | — | 22 | ~95% |

---

## Progetto completato!

Tutte le 6 fasi sono state completate, coprendo **tutte le competenze richieste da ALTEN**:

| Fase | Competenza ALTEN | Tecnologie |
|------|-----------------|------------|
| 1 | Requirements & System Engineering | SRS, casi d'uso, matrice tracciabilità |
| 2 | SW Embedded + FW | C, scheduler, driver sensori, protocollo binario |
| 3 | SW OOP | C++, Newton-Euler, RK4, extern "C" |
| 4 | SW OOP (Python) | PyQt6, matplotlib, parsing stream |
| 5 | Model Based Design | Octave, PID, Bode, export gains |
| 6 | HW Design + V&V | KiCad, I2C/SPI/UART, test unitari C + Python |
