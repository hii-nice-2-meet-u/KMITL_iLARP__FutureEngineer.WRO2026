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