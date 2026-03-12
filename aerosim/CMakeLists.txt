cmake_minimum_required(VERSION 3.20)
project(AeroSim VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Flags di compilazione
add_compile_options(-Wall -Wextra -Wpedantic)

# Build in modalità Debug di default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# Abilita testing
enable_testing()

# Sottomoduli
add_subdirectory(firmware)
add_subdirectory(physics)
add_subdirectory(tests)
