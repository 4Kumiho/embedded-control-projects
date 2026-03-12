# SimpleThermometer CMakeLists.txt
# Build configuration for firmware and tests

cmake_minimum_required(VERSION 3.15)
project(SimpleThermometer C)

# ============================================================================
#   COMPILER SETTINGS
# ============================================================================

# Set C standard to C99
set(CMAKE_C_STANDARD 99)

# Set C standard as required (not optional)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Enable all warnings
if(MSVC)
    # MSVC (Windows)
    add_compile_options(/W4)
else()
    # GCC/Clang (Linux/macOS)
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
#   FIRMWARE EXECUTABLE
# ============================================================================

# Create executable from firmware source files
add_executable(firmware_main
    firmware/main.c
    firmware/sensor.c
    firmware/serial.c
)

# Link math library (required for sin(), cos(), fabs())
target_link_libraries(firmware_main PRIVATE m)

# Include directories for header files
target_include_directories(firmware_main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/firmware
)

# ============================================================================
#   TEST EXECUTABLES
# ============================================================================

# Enable testing
enable_testing()

# Test 1: Sensor Tests
add_executable(test_sensor
    tests/test_sensor.c
    firmware/sensor.c
)
target_link_libraries(test_sensor PRIVATE m)
target_include_directories(test_sensor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
)
add_test(NAME SensorTests COMMAND test_sensor)

# Test 2: Serial Parser Tests
add_executable(test_parser
    tests/test_serial_parser.c
)
target_link_libraries(test_parser PRIVATE m)
target_include_directories(test_parser PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
)
add_test(NAME ParserTests COMMAND test_parser)

# ============================================================================
#   BUILD OUTPUT
# ============================================================================

# Message for user
message(STATUS "SimpleThermometer Build Configuration:")
message(STATUS "  C Standard: C99")
message(STATUS "  Firmware: firmware_main")
message(STATUS "  Tests: test_sensor, test_parser")
message(STATUS "")
message(STATUS "Build with:")
message(STATUS "  mkdir build && cd build && cmake .. && cmake --build .")
message(STATUS "")
message(STATUS "Run tests with:")
message(STATUS "  ctest --verbose")
