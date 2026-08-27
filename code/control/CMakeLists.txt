add_library(control STATIC
    pid.cpp
    stanley_controller.cpp
)

target_include_directories(control PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)