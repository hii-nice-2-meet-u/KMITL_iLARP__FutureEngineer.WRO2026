#include <iostream>

#include "lidar_module.hpp"
#include "lidar_processor.hpp"

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
		TimedLidarData scan;
		lidar::LidarProcessor lidar_processor;

		if (!lidar.wait_for_data(scan)) {
			std::cerr << "Failed to get LiDAR data\n";
			break;
		}
		float sum_distance = 0.0f;
		std::size_t count = 0;

		for (const auto &point : scan.points) {
			if (point.angle_deg >= 85.0f && point.angle_deg <= 95.0f &&
				point.distance_m > 0.0f) {

				sum_distance += point.distance_m;
				++count;
			}
		}
		if (count > 0) {
			const float avg_distance = sum_distance / static_cast<float>(count);

			std::cout << "Raw 90 deg AVG" << " | Distance: " << avg_distance
					  << " m" << " | Samples: " << count << '\n';
		} else {
			std::cout << "Raw 90 deg AVG | No valid points\n";
		}

		if (!scan.points.empty()) {
			// const auto &point = scan.points[scan.points.size() / 4];
			// const auto &point = scan.points.back();
			const lidar::ProcessedLidarData processed =
				lidar_processor.process(scan);


							 std::cout
					  << "Frame: " << frame
					  << " | Points: " << scan.points.size()
					  << " | Timestamp: " << scan.timestamp_us << '\n';

			std::cout << "Distance"
					  << " | Front: " << processed.front_distance_m << " m"
					  << " | Left: " << processed.left_distance_m << " m"
					  << " | Right: " << processed.right_distance_m << " m\n";

			std::cout << "Left wall"
					  << " | Valid: " << processed.left_wall.valid
					  << " | Distance: " << processed.left_wall.distance_m
					  << " m" << " | Angle: " << processed.left_wall.angle_deg
					  << " deg\n";

			std::cout << "Right wall"
					  << " | Valid: " << processed.right_wall.valid
					  << " | Distance: " << processed.right_wall.distance_m
					  << " m" << " | Angle: " << processed.right_wall.angle_deg
					  << " deg\n";

			std::cout << "Front wall"
					  << " | Valid: " << processed.front_wall.valid
					  << " | Distance: " << processed.front_wall.distance_m
					  << " m" << " | Angle: " << processed.front_wall.angle_deg
					  << " deg\n\n";
		}
	}
	lidar.stop();

	return 0;
}