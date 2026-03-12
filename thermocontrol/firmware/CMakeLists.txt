# Firmware library (C)
add_library(thermocontrol_fw
    main.c
    sensors.c
    pid_controller.c
    telemetry.c
    scheduler.c
    platform.c
)

target_include_directories(thermocontrol_fw PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Optional: physics integration (for simulation)
if(BUILD_PHYSICS)
    target_link_libraries(thermocontrol_fw PUBLIC thermocontrol_physics)
    target_compile_definitions(thermocontrol_fw PRIVATE ENABLE_PHYSICS_SIM)
endif()

# Firmware executable (simulator on PC)
add_executable(thermocontrol_sim main.c)
target_link_libraries(thermocontrol_sim PUBLIC thermocontrol_fw)

if(UNIX)
    target_link_libraries(thermocontrol_sim PUBLIC m)  # math lib
endif()
