add_executable(navigation_corner_sim_test
    navigation_corner_sim_test.cpp
)

target_link_libraries(navigation_corner_sim_test PRIVATE
    navigation
)

target_include_directories(navigation_corner_sim_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
)

add_test(NAME navigation_corner_sim_test
    COMMAND navigation_corner_sim_test
)

install(TARGETS navigation_corner_sim_test DESTINATION tests)

add_executable(navigation_heading_hold_test
    navigation_heading_hold_test.cpp
)

target_link_libraries(navigation_heading_hold_test PRIVATE
    navigation
)

target_include_directories(navigation_heading_hold_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
)

add_test(NAME navigation_heading_hold_test
    COMMAND navigation_heading_hold_test
)

install(TARGETS navigation_heading_hold_test DESTINATION tests)

add_executable(track_map_test
    track_map_test.cpp
)

target_link_libraries(track_map_test PRIVATE
    navigation
)

add_test(NAME track_map_test
    COMMAND track_map_test
)

install(TARGETS track_map_test DESTINATION tests)

add_executable(kinematics_test
    kinematics_test.cpp
)

target_include_directories(kinematics_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
)

add_test(NAME kinematics_test
    COMMAND kinematics_test
)

install(TARGETS kinematics_test DESTINATION tests)

add_executable(clothoid_reference_test
    clothoid_reference_test.cpp
)

target_include_directories(clothoid_reference_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
)

add_test(NAME clothoid_reference_test
    COMMAND clothoid_reference_test
)

install(TARGETS clothoid_reference_test DESTINATION tests)

add_executable(corner_planner_test
    corner_planner_test.cpp
)

target_include_directories(corner_planner_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${CMAKE_CURRENT_SOURCE_DIR}/../modules/navigation
)

add_test(NAME corner_planner_test
    COMMAND corner_planner_test
)

install(TARGETS corner_planner_test DESTINATION tests)

add_executable(servo_pulse_test
    servo_pulse_test.cpp
)

target_include_directories(servo_pulse_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
)

add_test(NAME servo_pulse_test
    COMMAND servo_pulse_test
)

install(TARGETS servo_pulse_test DESTINATION tests)

add_executable(lidar_deskew_test
    lidar_deskew_test.cpp
)

target_link_libraries(lidar_deskew_test PRIVATE
    lidar_processor
)

add_test(NAME lidar_deskew_test
    COMMAND lidar_deskew_test
)

install(TARGETS lidar_deskew_test DESTINATION tests)

add_executable(lidar_robust_fit_test
    lidar_robust_fit_test.cpp
)

target_link_libraries(lidar_robust_fit_test PRIVATE
    lidar_processor
)

add_test(NAME lidar_robust_fit_test
    COMMAND lidar_robust_fit_test
)

install(TARGETS lidar_robust_fit_test DESTINATION tests)

add_executable(perception_fusion_test
    perception_fusion_test.cpp
)

target_link_libraries(perception_fusion_test PRIVATE
    perception
)

add_test(NAME perception_fusion_test
    COMMAND perception_fusion_test
)

install(TARGETS perception_fusion_test DESTINATION tests)
