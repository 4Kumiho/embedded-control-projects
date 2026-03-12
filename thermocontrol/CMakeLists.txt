cmake_minimum_required(VERSION 3.15)
project(ThermoControl
    VERSION 1.0.0
    LANGUAGES C CXX
)

# Build options
option(BUILD_TESTS "Build test suite" ON)
option(BUILD_PHYSICS "Build physics simulator" ON)

# Compiler flags
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Warnings as errors for cleaner code
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Subdirectories
add_subdirectory(firmware)

if(BUILD_PHYSICS)
    add_subdirectory(physics)
endif()

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Print configuration
message(STATUS "=== ThermoControl Configuration ===")
message(STATUS "C Standard: ${CMAKE_C_STANDARD}")
message(STATUS "CXX Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Tests: ${BUILD_TESTS}")
message(STATUS "Build Physics: ${BUILD_PHYSICS}")
message(STATUS "=====================================")
