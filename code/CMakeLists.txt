cmake_minimum_required(VERSION 3.10)
project(iLARP-WRO-FE-2026 LANGUAGES CXX)

enable_testing()

set(CMAKE_CXX_STANDARD 17)

execute_process(
    COMMAND git rev-parse --git-path HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/..
    OUTPUT_VARIABLE ILARP_GIT_HEAD_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
    COMMAND git rev-parse --git-path index
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/..
    OUTPUT_VARIABLE ILARP_GIT_INDEX_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
    COMMAND git symbolic-ref -q HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/..
    OUTPUT_VARIABLE ILARP_GIT_SYMBOLIC_REF
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT ILARP_GIT_SYMBOLIC_REF STREQUAL "")
    execute_process(
        COMMAND git rev-parse --git-path ${ILARP_GIT_SYMBOLIC_REF}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/..
        OUTPUT_VARIABLE ILARP_GIT_REF_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif()

foreach(GIT_STATE_PATH IN ITEMS
        "${ILARP_GIT_HEAD_PATH}" "${ILARP_GIT_INDEX_PATH}"
        "${ILARP_GIT_REF_PATH}")
    if(NOT GIT_STATE_PATH STREQUAL "")
        get_filename_component(GIT_STATE_PATH_ABSOLUTE
            "${GIT_STATE_PATH}" ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}/..")
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
            "${GIT_STATE_PATH_ABSOLUTE}")
    endif()
endforeach()

execute_process(
    COMMAND git rev-parse HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/..
    OUTPUT_VARIABLE ILARP_GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE ILARP_GIT_HASH_RESULT
)
if(NOT ILARP_GIT_HASH_RESULT EQUAL 0)
    set(ILARP_GIT_COMMIT_HASH "UNKNOWN")
endif()

execute_process(
    COMMAND git status --porcelain --untracked-files=normal
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/..
    OUTPUT_VARIABLE ILARP_GIT_STATUS
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE ILARP_GIT_STATUS_RESULT
)
if(ILARP_GIT_STATUS_RESULT EQUAL 0 AND ILARP_GIT_STATUS STREQUAL "")
    set(ILARP_GIT_DIRTY 0)
else()
    set(ILARP_GIT_DIRTY 1)
endif()

string(TIMESTAMP ILARP_BUILD_TIMESTAMP "%Y-%m-%dT%H:%M:%SZ" UTC)
add_compile_definitions(
    ILARP_GIT_COMMIT_HASH="${ILARP_GIT_COMMIT_HASH}"
    ILARP_GIT_DIRTY=${ILARP_GIT_DIRTY}
    ILARP_BUILD_TIMESTAMP="${ILARP_BUILD_TIMESTAMP}"
)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

if(LIBCAMERA_FOUND)
    add_subdirectory(external/LCCV)
endif()

add_subdirectory(modules/lidar)
add_subdirectory(modules/camera)
add_subdirectory(modules/otos)
add_subdirectory(modules/navigation)
add_subdirectory(modules/perception)
add_subdirectory(modules/spi)
add_subdirectory(modules/logging)
add_subdirectory(control)
add_subdirectory(tests)

add_subdirectory(app/test_camera)
add_subdirectory(app/test_lidar)
add_subdirectory(app/test_perception)
add_subdirectory(app/test_otos)
add_subdirectory(app/measure_otos_scale)
add_subdirectory(app/test_spi)
add_subdirectory(app/camera_tuner)
add_subdirectory(app/_challenge/open)
add_subdirectory(app/_challenge/obstacle)
