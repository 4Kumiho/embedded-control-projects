# SimpleThermometer Compilation Guide

## Prerequisites

```bash
# Linux/macOS
sudo apt-get install gcc make cmake
# or
brew install gcc cmake

# Windows (MSYS2 or MinGW)
Download MSYS2: https://www.msys2.org/
pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-cmake
```

## Build with CMake (Recommended)

```bash
# Create build directory
mkdir build
cd build

# Configure (generate Makefiles)
cmake ..

# Build
cmake --build .

# Run tests
ctest --verbose
```

## Build with Make (Direct)

```bash
# Compile firmware
gcc -o firmware_main firmware/main.c firmware/sensor.c firmware/serial.c -lm

# Compile sensor tests
gcc -o test_sensor tests/test_sensor.c firmware/sensor.c -lm

# Compile parser tests
gcc -o test_parser tests/test_serial_parser.c

# Run firmware
./firmware_main

# Run tests
./test_sensor
./test_parser
```

## Expected Output

### Firmware (firmware_main)

```
========================================
SimpleThermometer v1.0
Reading temperature every 500ms
========================================
TEMP:25.3°C
TEMP:24.8°C
TEMP:25.1°C
TEMP:24.9°C
TEMP:25.2°C
(continues indefinitely until interrupted)
```

### Sensor Tests (test_sensor)

```
==========================================
SimpleThermometer - Sensor Tests
==========================================
▶ sensor_init_succeeds
  ✓ PASS
▶ sensor_returns_valid_range
  ✓ PASS
▶ sensor_returns_valid_float
  ✓ PASS
▶ sensor_produces_variation
  ✓ PASS
▶ sensor_changes_smoothly
  ✓ PASS

================================
Test Results:
  Passed: 5 / 5
  Failed: 0 / 5
================================
```

### Parser Tests (test_parser)

```
==========================================
SimpleThermometer - Serial Parser Tests
==========================================
▶ parser_valid_temperature
  ✓ PASS
▶ parser_boundary_values
  ✓ PASS
▶ parser_different_precision
  ✓ PASS
▶ parser_rejects_invalid
  ✓ PASS

================================
Test Results:
  Passed: 4 / 4
  Failed: 0 / 4
================================
```

## Python GUI

```bash
# Install dependencies
pip install -r ground_station/requirements.txt

# Run GUI
python ground_station/main.py
```

Expected output:
```
==================================================
SimpleThermometer - Ground Station
==================================================

[SerialReader] Attempting to connect to COM3...
[SerialReader] PySerial not installed, using simulated data
[SerialReader] Thread started
```

Then a PyQt6 window opens showing real-time temperature plot.

## Compilation Flags

For debug builds:
```bash
gcc -g -O0 -o firmware_main firmware/*.c -lm
```

For optimized builds:
```bash
gcc -O2 -DNDEBUG -o firmware_main firmware/*.c -lm
```

For strict warnings:
```bash
gcc -Wall -Wextra -Wpedantic -pedantic-errors -o firmware_main firmware/*.c -lm
```

## Troubleshooting

| Error | Solution |
|-------|----------|
| `gcc: command not found` | Install GCC (see Prerequisites) |
| `fatal error: unistd.h: No such file or directory` | Using Windows without MSYS2/MinGW. Use CMake instead. |
| `-lm: command not found` | Add `-lm` flag at end (math library) |
| Port COM3 not found | Edit `ground_station/main.py` to use correct port |

## File Sizes

```
firmware/main.c          ~250 lines
firmware/sensor.c        ~130 lines
firmware/serial.c        ~80 lines
firmware/sensor.h        ~30 lines
firmware/serial.h        ~30 lines
Total Firmware: ~520 lines

ground_station/main.py         ~400 lines
ground_station/serial_reader.py ~310 lines
Total Python: ~710 lines

tests/test_framework.h     ~150 lines
tests/test_sensor.c        ~180 lines
tests/test_serial_parser.c ~180 lines
Total Tests: ~510 lines

docs/                      ~2000 lines

TOTAL:                     ~3700 lines (including docs)
```

---
