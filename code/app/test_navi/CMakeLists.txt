add_executable(test_navi main.cpp)

target_link_libraries(test_navi PRIVATE
	lidar_module
	lidar_processor
	navigation
)

install(TARGETS test_navi DESTINATION app)