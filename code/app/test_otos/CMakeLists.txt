add_executable(test_otos main.cpp)

target_include_directories(test_otos PRIVATE)

target_link_libraries(test_otos PRIVATE otos)

install(TARGETS test_otos DESTINATION app)