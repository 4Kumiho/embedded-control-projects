add_library(physics STATIC
    Environment.cpp
    Drone.cpp
    drone_c_api.cpp
)

target_include_directories(physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# C++ richiede la libreria math
target_link_libraries(physics PUBLIC m)
