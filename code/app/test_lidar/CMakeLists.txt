add_executable(test_lidar  main.cpp)

target_include_directories(test_lidar PRIVATE)

target_link_libraries(test_lidar PRIVATE lidar_module lidar_processor otos navigation logging)

install(TARGETS test_lidar DESTINATION app)
