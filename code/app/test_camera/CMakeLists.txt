add_executable(test_camera  main.cpp)

target_include_directories(test_camera PRIVATE
    ${CMAKE_SOURCE_DIR}/external/LCCV/include
    ${OpenCV_INCLUDE_DIRS}
    ${LIBCAMERA_INCLUDE_DIRS}
)

target_link_libraries(test_camera PRIVATE
    liblccv
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
)

install(TARGETS test_camera DESTINATION app/test_camera)