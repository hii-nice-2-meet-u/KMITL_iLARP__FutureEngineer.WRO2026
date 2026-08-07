cmake_minimum_required(VERSION 3.10)
project(iLARP-WRO-FE-2026 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

if(LIBCAMERA_FOUND)
    add_subdirectory(external/LCCV)
endif()

set(RPLIDAR_SDK_PATH ${CMAKE_SOURCE_DIR}/external/rplidar_sdk/sdk)

file(GLOB_RECURSE RPLIDAR_SDK_SOURCES ${RPLIDAR_SDK_PATH}/src/*.cpp)

list(FILTER RPLIDAR_SDK_SOURCES EXCLUDE REGEX ".*macOS.*")
list(FILTER RPLIDAR_SDK_SOURCES EXCLUDE REGEX ".*win32.*")

add_library(rplidar_sdk STATIC ${RPLIDAR_SDK_SOURCES})

target_include_directories(rplidar_sdk PUBLIC 
    ${RPLIDAR_SDK_PATH}/include
    ${RPLIDAR_SDK_PATH}/src
)

target_link_libraries(rplidar_sdk PUBLIC pthread)

add_subdirectory(app/test_camera)
add_subdirectory(app/test_lidar)
