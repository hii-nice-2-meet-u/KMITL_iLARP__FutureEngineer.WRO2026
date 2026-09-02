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
