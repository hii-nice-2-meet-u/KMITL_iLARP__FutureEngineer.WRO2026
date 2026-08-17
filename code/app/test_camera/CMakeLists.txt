add_executable(test_camera  main.cpp)

target_include_directories(test_camera PRIVATE)

target_link_libraries(test_camera PRIVATE camera_module camera_processor)

set_target_properties(test_camera PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib")

install(TARGETS test_camera DESTINATION app)