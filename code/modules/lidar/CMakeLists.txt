# NOTE: lidar_module

# Building rplidar_sdk
set(RPLIDAR_SDK_PATH ${CMAKE_SOURCE_DIR}/external/rplidar_sdk/sdk)
file(GLOB_RECURSE RPLIDAR_SDK_SOURCES ${RPLIDAR_SDK_PATH}/src/*.cpp)
list(FILTER RPLIDAR_SDK_SOURCES EXCLUDE REGEX ".*macOS.*")
list(FILTER RPLIDAR_SDK_SOURCES EXCLUDE REGEX ".*win32.*")
add_library(rplidar_sdk STATIC ${RPLIDAR_SDK_SOURCES})
target_include_directories(rplidar_sdk PUBLIC ${RPLIDAR_SDK_PATH}/include
                                              ${RPLIDAR_SDK_PATH}/src)

add_library(lidar_module STATIC lidar_module.cpp lidar_module.hpp)
target_include_directories(lidar_module PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
                                               ${CMAKE_SOURCE_DIR}/src/types)
target_link_libraries(lidar_module PUBLIC rplidar_sdk )