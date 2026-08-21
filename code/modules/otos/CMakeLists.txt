# Target: otos_module

add_library(otos STATIC
    otos.cpp
    otos.hpp
    linux_i2c.cpp
    linux_i2c.hpp

    external/OTOS/src/sfTk/sfDevOTOS.cpp
)

target_include_directories(otos PUBLIC
    ${CMAKE_SOURCE_DIR}/modules/otos

    ${CMAKE_SOURCE_DIR}/external/OTOS/src
    ${CMAKE_SOURCE_DIR}/external/SparkFunToolkit/src
)