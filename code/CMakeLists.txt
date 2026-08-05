cmake_minimum_required(VERSION 3.10)
project(iLARP-WRO-FE-2026 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBCAMERA REQUIRED libcamera)

if(LIBCAMERA_FOUND)
    add_subdirectory(lib/LCCV)
endif()

add_subdirectory(app/test_camera)