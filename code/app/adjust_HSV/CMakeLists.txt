add_executable(adjust_HSV main.cpp)

target_include_directories(adjust_HSV PRIVATE
    ${CMAKE_SOURCE_DIR}/external/LCCV/include
    ${OpenCV_INCLUDE_DIRS}
    ${LIBCAMERA_INCLUDE_DIRS}
)

target_link_libraries(adjust_HSV PRIVATE
    liblccv
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
)

set_target_properties(adjust_HSV PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")

install(TARGETS adjust_HSV DESTINATION app)