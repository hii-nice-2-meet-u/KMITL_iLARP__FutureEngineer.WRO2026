add_executable(obstacle_challenge_main
    main.cpp
)

add_executable(obstacle_challenge_main2
    main2.cpp
)

target_include_directories(obstacle_challenge_main PRIVATE
    ${CMAKE_SOURCE_DIR}/app/_challenge/open
)

target_include_directories(obstacle_challenge_main2 PRIVATE
    ${CMAKE_SOURCE_DIR}/app/_challenge/open
)

target_link_libraries(obstacle_challenge_main PRIVATE
    camera_module
    camera_processor
    lidar_module
    lidar_processor
    perception
    navigation
    otos
    spi_master
    logging
)

target_link_libraries(obstacle_challenge_main2 PRIVATE
    camera_module
    camera_processor
    lidar_module
    lidar_processor
    perception
    navigation
    otos
    logging
)

set_target_properties(obstacle_challenge_main PROPERTIES
    INSTALL_RPATH "$ORIGIN/../../lib")

set_target_properties(obstacle_challenge_main2 PROPERTIES
    INSTALL_RPATH "$ORIGIN/../../lib")

install(TARGETS
    obstacle_challenge_main
    obstacle_challenge_main2
    DESTINATION challenge/obstacle
)
