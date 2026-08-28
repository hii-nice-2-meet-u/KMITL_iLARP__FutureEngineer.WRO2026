add_executable(test_spi
    main.cpp
)

target_link_libraries(test_spi PRIVATE
    spi_master
)

install(TARGETS test_spi DESTINATION app)
