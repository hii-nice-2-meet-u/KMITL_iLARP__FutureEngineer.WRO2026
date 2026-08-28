add_library(spi_master STATIC
    spi_master.cpp
)

target_include_directories(spi_master PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
