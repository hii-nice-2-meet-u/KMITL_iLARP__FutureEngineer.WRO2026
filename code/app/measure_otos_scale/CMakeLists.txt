add_executable(measure_otos_scale main.cpp)

target_link_libraries(measure_otos_scale PRIVATE otos)

install(TARGETS measure_otos_scale DESTINATION app)
