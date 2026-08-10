#include <chrono>
#include <iostream>
#include <thread>

#include "lidar_module.hpp"

int main() {
	lidar::LidarModule lidar;

	if (!lidar.initialize()) {
		std::cerr << "Initialize failed\n";
		return -1;
	}

	if (!lidar.start()) {
		std::cerr << "Start failed\n";
		return -1;
	}

	for (int i = 0; i < 10; ++i) {

		std::this_thread::sleep_for(std::chrono::microseconds(250));
		TimedLidarData data;
		if (!lidar.get_latest(data)) {
			std::cout << "No LiDAR data yet\n";
			continue;
		}
		std::cout << "Points: " << data.points.size()
				  << " | Timestamp: " << data.timestamp_us << '\n';

		if (!data.points.empty()) {
			const auto &point = data.points.front();

			std::cout << "  First point:" << " angle=" << point.angle_deg
					  << " deg" << " distance=" << point.distance_mm << " m"
					  << " quality=" << static_cast<int>(point.quality) << '\n';
		}
	}

	lidar.stop();
	lidar.shutdown();

	std::cout << "Finished\n";

	return 0;
}