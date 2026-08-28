add_executable(navigation_corner_sim_test
    navigation_corner_sim_test.cpp
)

target_link_libraries(navigation_corner_sim_test PRIVATE
    navigation
)

add_test(NAME navigation_corner_sim_test
    COMMAND navigation_corner_sim_test
)

install(TARGETS navigation_corner_sim_test DESTINATION app)

add_executable(navigation_heading_hold_test
    navigation_heading_hold_test.cpp
)

target_link_libraries(navigation_heading_hold_test PRIVATE
    navigation
)

add_test(NAME navigation_heading_hold_test
    COMMAND navigation_heading_hold_test
)

install(TARGETS navigation_heading_hold_test DESTINATION app)

add_executable(track_map_test
    track_map_test.cpp
)

target_link_libraries(track_map_test PRIVATE
    navigation
)

add_test(NAME track_map_test
    COMMAND track_map_test
)

install(TARGETS track_map_test DESTINATION app)
