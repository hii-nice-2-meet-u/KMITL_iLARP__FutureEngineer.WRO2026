add_executable(test_perception
    main.cpp
)

target_link_libraries(test_perception PRIVATE
    camera_module
    camera_processor
    lidar_module
    lidar_processor
    perception
)

set_target_properties(test_perception PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
)

install(TARGETS test_perception DESTINATION app)
