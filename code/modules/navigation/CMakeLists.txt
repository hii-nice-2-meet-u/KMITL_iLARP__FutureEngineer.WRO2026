add_library(navigation STATIC
    init_direction.cpp
    navigation_controller.cpp
    track_map.cpp
)

target_include_directories(navigation PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/common
)

target_link_libraries(navigation PUBLIC
    lidar_processor
    control
)
