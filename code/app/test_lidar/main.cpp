#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <optional>

#include <opencv2/opencv.hpp>

#include "lidar_module.hpp"
#include "lidar_processor.hpp"

int main() {
	// lidar::LidarModule lidar;
	lidar::LidarModule lidar("/dev/ttyAMA0", 1000000);
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

	// +X = right
	// +Y = front
	auto world_to_pixel = [&](const cv::Point2f &point_m) -> cv::Point {
		const float pixel_x = origin.x + point_m.x * SCALE_PX_PER_M;

		const float pixel_y = origin.y - point_m.y * SCALE_PX_PER_M;

		return cv::Point(static_cast<int>(std::lround(pixel_x)),
			static_cast<int>(std::lround(pixel_y)));
	};

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

		// -----------------------------
		// Process LiDAR
		// -----------------------------

		const auto process_start = std::chrono::steady_clock::now();

		const lidar::ProcessedLidarData processed =
			lidar_processor.process(scan);

		const auto process_end = std::chrono::steady_clock::now();

		const auto process_us =
			std::chrono::duration_cast<std::chrono::microseconds>(
				process_end - process_start)
				.count();

		// -----------------------------
		// Frame interval
		// -----------------------------

		std::uint64_t frame_diff_us = 0;

		if (timestamp_prev != 0) {
			frame_diff_us = processed.timestamp_us - timestamp_prev;
		}

		timestamp_prev = processed.timestamp_us;

		// -----------------------------
		// Clear map
		// -----------------------------

		debug_map.setTo(cv::Scalar(0, 0, 0));

		// -----------------------------
		// All LineSegments
		// Gray = raw geometry
		// -----------------------------

		for (const auto &segment : processed.line_segments) {

			cv::line(debug_map, world_to_pixel(segment.start),
				world_to_pixel(segment.end), cv::Scalar(80, 80, 80), 1,
				cv::LINE_AA);
		}

		// -----------------------------
		// Resolved track walls
		// Green
		// -----------------------------

		auto draw_wall = [&](const std::optional<lidar::LineSegment> &wall) {
			if (!wall.has_value()) {
				return;
			}

			cv::line(debug_map, world_to_pixel(wall->start),
				world_to_pixel(wall->end), cv::Scalar(0, 255, 0), 3,
				cv::LINE_AA);
		};

		draw_wall(processed.walls.left);
		draw_wall(processed.walls.right);
		draw_wall(processed.walls.front);

		// -----------------------------
		// Obstacles
		// Red
		// -----------------------------

		for (const auto &obstacle : processed.obstacles) {
			const auto obj_start = obstacle.start();
			const auto obj_end = obstacle.end();
			cv::line(debug_map, world_to_pixel(obj_start),
				world_to_pixel(obj_end), cv::Scalar(0, 0, 255), 4,
				cv::LINE_AA);

			auto const &center = obstacle.center;

			// Obstacle center
			cv::circle(debug_map, world_to_pixel(center), 4,
				cv::Scalar(0, 255, 255), -1);

			// Distance + bearing
			auto const &distance_m = obstacle.distance_m();

			auto const &bearing_rad = obstacle.bearing_rad();

			const float bearing_deg =
				bearing_rad * 180.0f / static_cast<float>(M_PI);

			const std::string obstacle_info =
				"D:" + std::to_string(distance_m).substr(0, 4) +
				" B:" + std::to_string(bearing_deg).substr(0, 5);

			cv::putText(debug_map, obstacle_info,
				world_to_pixel(center) + cv::Point(8, -8),
				cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1,
				cv::LINE_AA);
		}

		// -----------------------------
		// LiDAR origin
		// Blue
		// -----------------------------

		cv::circle(debug_map,
			cv::Point(static_cast<int>(origin.x), static_cast<int>(origin.y)),
			5, cv::Scalar(255, 0, 0), -1);

		// Front direction indicator
		cv::line(debug_map,
			cv::Point(static_cast<int>(origin.x), static_cast<int>(origin.y)),
			cv::Point(
				static_cast<int>(origin.x), static_cast<int>(origin.y - 30)),
			cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

		// -----------------------------
		// Count resolved walls
		// -----------------------------

		int wall_count = 0;

		if (processed.walls.left.has_value()) {
			++wall_count;
		}

		if (processed.walls.right.has_value()) {
			++wall_count;
		}

		if (processed.walls.front.has_value()) {
			++wall_count;
		}

		// -----------------------------
		// Debug info
		// -----------------------------

		const std::string info =
			"Points: " + std::to_string(scan.points.size()) +
			"  Segments: " + std::to_string(processed.line_segments.size()) +
			"  Walls: " + std::to_string(wall_count) +
			"  Obstacles: " + std::to_string(processed.obstacles.size());

		cv::putText(debug_map, info, cv::Point(10, 25),
			cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
			cv::LINE_AA);

		const std::string timing =
			"Process: " + std::to_string(process_us / 1000.0) +
			" ms  Frame: " + std::to_string(frame_diff_us / 1000.0) + " ms";

		cv::putText(debug_map, timing, cv::Point(10, 50),
			cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
			cv::LINE_AA);

		// -----------------------------
		// Show
		// -----------------------------

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