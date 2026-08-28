add_executable(open_challenge_main1
    main1.cpp
)

target_link_libraries(open_challenge_main1 PRIVATE
    lidar_module
    lidar_processor
    navigation
    otos
)

add_executable(open_challenge_main2
    main2.cpp
)

target_link_libraries(open_challenge_main2 PRIVATE
    lidar_module
    lidar_processor
    navigation
    otos
)

install(TARGETS
    open_challenge_main1
    open_challenge_main2
    DESTINATION app
)
