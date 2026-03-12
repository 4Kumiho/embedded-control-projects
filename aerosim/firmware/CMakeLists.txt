# Libreria statica firmware (usata anche dai test)
add_library(firmware STATIC
    sensors.c
    scheduler.c
    telemetry.c
)

target_include_directories(firmware PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Linka la libreria math (richiesta da sensors.c per sqrt/log/cos)
target_link_libraries(firmware PUBLIC m)

# Eseguibile del simulatore firmware
add_executable(firmware_sim main.c)
target_link_libraries(firmware_sim PRIVATE firmware)
