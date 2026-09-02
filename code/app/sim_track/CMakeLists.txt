add_executable(sim_track main.cpp)

# Reuses the real navigation/lidar/logging libraries. The open-challenge headers
# (make_navigation_config, ActuatorConfig) pull otos/lidar_module/spi headers for
# declarations only -- those symbols are never odr-used here, so no hardware
# library is linked.
target_include_directories(sim_track PRIVATE
    ${CMAKE_SOURCE_DIR}/app/_challenge/open
    ${CMAKE_SOURCE_DIR}/modules/otos
    ${CMAKE_SOURCE_DIR}/external/OTOS/src
    ${CMAKE_SOURCE_DIR}/external/SparkFunToolkit/src
    ${CMAKE_SOURCE_DIR}/modules/lidar
    ${CMAKE_SOURCE_DIR}/external/rplidar_sdk/sdk/include
    ${CMAKE_SOURCE_DIR}/modules/spi
)

target_link_libraries(sim_track PRIVATE
    navigation
    lidar_processor
    logging
    control
)

install(TARGETS sim_track DESTINATION app)
