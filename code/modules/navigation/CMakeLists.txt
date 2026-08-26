add_library(navigation STATIC
    initial_direction_estimator.cpp
    initial_direction_estimator.hpp
)
target_include_directories(navigation PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/common
)

target_link_libraries(navigation PRIVATE 
	lidar_processor)