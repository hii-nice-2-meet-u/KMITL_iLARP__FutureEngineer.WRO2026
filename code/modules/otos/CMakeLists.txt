# Target: otos

add_library(otos STATIC
    otos.cpp
    otos.hpp
    linux_i2c.cpp
    linux_i2c.hpp

    ${CMAKE_SOURCE_DIR}/external/OTOS/src/sfTk/sfDevOTOS.cpp
    ${CMAKE_SOURCE_DIR}/external/SparkFunToolkit/src/sfTk/sfToolkit.cpp
)

target_include_directories(otos PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/external/OTOS/src
    ${CMAKE_SOURCE_DIR}/external/SparkFunToolkit/src
)