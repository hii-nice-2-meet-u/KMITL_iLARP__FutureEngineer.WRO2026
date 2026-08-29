find_package(Threads REQUIRED)

add_library(logging STATIC
    async_csv_writer.cpp
    log_types.cpp
    telemetry_logger.cpp
    wall_logger.cpp
)

target_include_directories(logging PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(logging PUBLIC
    lidar_processor
    navigation
    Threads::Threads
)
