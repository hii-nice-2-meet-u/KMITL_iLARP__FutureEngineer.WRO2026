#include <iostream>

#include "lidar_module.hpp"

int main() {
	lidar::LidarModule lidar;

	if (!lidar.initialize()) {
		std::cerr << "Initialize failed\n";
		return 1;
	}

	if (!lidar.start()) {
		std::cerr << "Start failed\n";
		return 1;
	}

	std::cout << "Waiting for LiDAR frames...\n";

	for (int frame = 0; frame < 20; ++frame) {
		TimedLidarData data;

		if (!lidar.wait_for_data(data)) {
			std::cerr << "Failed to get LiDAR data\n";
			break;
		}

		std::cout << "Frame: " << frame << " | Points: " << data.points.size();

		if (!data.points.empty()) {
			const auto &point = data.points.front();

			std::cout << " angle=" << point.angle_deg
					  << " deg" << " distance=" << point.distance_m
					  << " quality=" << static_cast<int>(point.quality) << '\n';
		}
	}

	lidar.stop();
	lidar.shutdown();

	std::cout << "Finished\n";

	return 0;
}