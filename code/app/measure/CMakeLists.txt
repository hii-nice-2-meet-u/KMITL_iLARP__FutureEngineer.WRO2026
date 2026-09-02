add_executable(measure main.cpp)

# lidar_processor is linked (not only lidar_module) so the common/ include path
# it exports brings in servo_pulse.hpp and kinematics.hpp.
target_link_libraries(measure PRIVATE
    otos
    spi_master
    lidar_module
    lidar_processor
)

install(TARGETS measure DESTINATION app)
