add_executable(camera_tuner
    main.cpp
    tunner.cpp
    tunner.hpp
)

target_include_directories(camera_tuner
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(camera_tuner
    PRIVATE
        camera_module
)

set_target_properties(camera_tuner PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
)

install(
    TARGETS camera_tuner
    DESTINATION app
)