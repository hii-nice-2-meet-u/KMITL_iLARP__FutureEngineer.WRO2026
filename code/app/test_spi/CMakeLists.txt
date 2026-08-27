add_executable(test_spi  main.cpp)

target_include_directories(test_spi PRIVATE)

target_link_libraries(test_spi PRIVATE)

install(TARGETS test_spi DESTINATION app)