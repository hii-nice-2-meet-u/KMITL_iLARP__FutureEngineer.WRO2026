add_library(navigation STATIC
    init_direction.cpp
    init_direction.hpp
)
target_include_directories(navigation PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/common
)

target_link_libraries(navigation PRIVATE 
	lidar_processor)