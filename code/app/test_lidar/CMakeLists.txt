add_executable(test_lidar  main.cpp)

target_include_directories(test_camera PRIVATE
    ${CMAKE_SOURCE_DIR}/external/rplidar_sdk/sdk/include
    ${CMAKE_SOURCE_DIR}/external/rplidar_sdk/sdk/src
)

target_link_libraries(test_lidar PRIVATE rplidar_sdk)

install(TARGETS test_lidar DESTINATION app/test_lidar)