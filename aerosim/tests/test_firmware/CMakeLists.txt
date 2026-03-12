add_executable(test_firmware
    test_runner.c
    test_sensors.c
    test_telemetry.c
    test_scheduler.c
)

target_include_directories(test_firmware PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/firmware
)

target_link_libraries(test_firmware PRIVATE firmware m)

# Registra il test nel framework CTest
add_test(NAME firmware_tests COMMAND test_firmware)
