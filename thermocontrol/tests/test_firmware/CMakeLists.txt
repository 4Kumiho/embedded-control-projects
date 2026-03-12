# Firmware unit tests (C)
add_executable(test_firmware
    test_runner.c
    test_sensors.c
    test_pid.c
    test_telemetry.c
    test_scheduler.c
)

target_include_directories(test_firmware PRIVATE ${CMAKE_SOURCE_DIR}/firmware)
target_link_libraries(test_firmware PRIVATE thermocontrol_fw)

if(UNIX)
    target_link_libraries(test_firmware PRIVATE m)
endif()

add_test(NAME FirmwareTests COMMAND test_firmware)
