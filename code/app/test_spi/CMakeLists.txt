add_executable(test_spi
    main.cpp
)

add_executable(test_spi_legacy
    main2.cpp
)

target_link_libraries(test_spi PRIVATE
    spi_master
)

install(TARGETS test_spi test_spi_legacy DESTINATION app)
