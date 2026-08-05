add_executable(test_camera  main.cpp)

target_include_directories(test_camera PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${LIBCAMERA_INCLUDE_DIRS}
)

target_link_libraries(test_camera PRIVATE
    lccv
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
    pthread
)