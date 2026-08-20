#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "lidar_module.hpp"
#include "lidar_processor.hpp"

int main() {
	lidar::LidarModule lidar;
	lidar::LidarProcessor lidar_processor;

	if (!lidar.initialize()) {
		std::cerr << "Initialize failed\n";
		return 1;
	}

	if (!lidar.start()) {
		std::cerr << "Start failed\n";
		return 1;
	}

	std::cout << "Waiting for LiDAR frames...\n";

	// Debug map settings
	constexpr int MAP_WIDTH = 800;
	constexpr int MAP_HEIGHT = 800;

	// 1 meter = 300 pixels
	constexpr float SCALE_PX_PER_M = 300.0f;

	cv::Mat debug_map(MAP_HEIGHT, MAP_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

	const cv::Point2f origin(static_cast<float>(MAP_WIDTH) * 0.5f,
		static_cast<float>(MAP_HEIGHT) * 0.5f);

	std::uint64_t timestamp_prev = 0;

	while (true) {
		TimedLidarData scan;

		if (!lidar.wait_for_data(scan)) {
			std::cerr << "Failed to get LiDAR data\n";
			break;
		}

		if (scan.points.empty()) {
			continue;
		}

		// Process LiDAR

		const auto process_start = std::chrono::steady_clock::now();

		const lidar::ProcessedLidarData processed =
			lidar_processor.process(scan);

		const auto process_end = std::chrono::steady_clock::now();

		const auto process_us =
			std::chrono::duration_cast<std::chrono::microseconds>(
				process_end - process_start)
				.count();

		// Calculate frame interval

		std::uint64_t frame_diff_us = 0;

		if (timestamp_prev != 0) {
			frame_diff_us = processed.timestamp_us - timestamp_prev;
		}

		timestamp_prev = processed.timestamp_us;

		// Clear previous frame

		debug_map.setTo(cv::Scalar(0, 0, 0));

		for (const auto &wall : processed.line_segments) {
			lidar_processor.draw_segment(debug_map, wall, SCALE_PX_PER_M);
		}

		cv::circle(debug_map,
			cv::Point(static_cast<int>(origin.x), static_cast<int>(origin.y)),
			5, cv::Scalar(0, 0, 255), -1);

		const std::string info =
			"Points: " + std::to_string(scan.points.size()) +
			"  Walls: " + std::to_string(processed.line_segments.size());

		cv::putText(debug_map, info, cv::Point(10, 25),
			cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
			cv::LINE_AA);

		const std::string timing =
			"Process: " + std::to_string(process_us / 1000.0) +
			" ms  Frame: " + std::to_string(frame_diff_us / 1000.0) + " ms";

		cv::putText(debug_map, timing, cv::Point(10, 50),
			cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
			cv::LINE_AA);

		// Show

		cv::imshow("LiDAR Debug Map", debug_map);

		const int key = cv::waitKey(1);

		if (key == 'q' || key == 27) {
			break;
		}
	}

	lidar.stop();

	cv::destroyAllWindows();

	return 0;
}