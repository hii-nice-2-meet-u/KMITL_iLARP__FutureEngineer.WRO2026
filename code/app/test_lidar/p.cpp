lude "lidar_module.hpp"

int main() {
	lidar::LidarModule lidar("/dev/ttyUSB0", 1000000);

	if (!lidar.initialize()) {

		std::cerr << "LiDAR initialize failed\n";

		return 1;
	}

	if (!lidar.start()) {

		std::cerr << "LiDAR start failed\n";

		return 1;
	}

	while (true) {
		TimedLidarData scan;
		if (!lidar.wait_for_data(scan)) {

			break;
		}

		if (scan.points.empty()) {
			continue;
		}

		std::cout << "Point:" << scan.points[0] << "\n";
	}

	lidar.stop();
	return 0;
}