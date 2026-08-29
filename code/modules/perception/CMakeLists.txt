add_library(perception STATIC
    perception.cpp
    perception.hpp
)

target_include_directories(perception PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(perception PUBLIC
    camera_processor
    lidar_processor
    navigation
)
