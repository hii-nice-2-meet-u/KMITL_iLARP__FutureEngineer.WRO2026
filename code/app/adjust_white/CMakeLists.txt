add_executable(adjust_white main.cpp)

target_include_directories(adjust_white PRIVATE
    ${CMAKE_SOURCE_DIR}/external/LCCV/include
    ${OpenCV_INCLUDE_DIRS}
    ${LIBCAMERA_INCLUDE_DIRS}
)

target_link_libraries(adjust_white PRIVATE
    liblccv
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
)

set_target_properties(adjust_white PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")

install(TARGETS adjust_white DESTINATION app)