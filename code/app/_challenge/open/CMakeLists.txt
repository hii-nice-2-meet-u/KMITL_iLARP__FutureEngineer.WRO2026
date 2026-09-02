
add_executable(open_challenge_main
    main.cpp
)

target_link_libraries(open_challenge_main PRIVATE
    PkgConfig::LIBGPIOD
)

target_link_libraries(open_challenge_main PRIVATE
    lidar_module
    lidar_processor
    navigation
    otos
    spi_master
    logging
)

install(TARGETS
    open_challenge_main
    DESTINATION challenge/open
)
