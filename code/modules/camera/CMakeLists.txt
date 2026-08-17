# NOTE: camera_module camera_processor camera struct

# Target: camera_processor
add_library(camera_processor STATIC
    camera_processor.cpp
    camera_processor.hpp
)

target_include_directories(camera_processor
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
		${OpenCV_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/utils
)

target_link_libraries(camera_processor
    PRIVATE
        ${OpenCV_LIBS}
)

# Target: camera_module

add_library(camera_module STATIC 
    camera_module.cpp 
    camera_module.hpp
)

target_include_directories(camera_module
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/external/LCCV/include
        ${OpenCV_INCLUDE_DIRS}
        ${LIBCAMERA_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/utils
)


target_link_libraries(camera_module
    PRIVATE
        ${OpenCV_LIBS}
        ${LIBCAMERA_LIBRARIES}
        liblccv
)    