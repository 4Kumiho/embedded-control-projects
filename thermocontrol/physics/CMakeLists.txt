# Physics simulator (C++)
add_library(thermocontrol_physics
    ThermoModel.cpp
    thermocontrol_c_api.cpp
)

target_include_directories(thermocontrol_physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Link standard C++ libraries
if(UNIX)
    target_link_libraries(thermocontrol_physics PUBLIC m)
endif()

# C interface
target_compile_features(thermocontrol_physics PUBLIC cxx_std_14)
