#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>

#include "lidar_module.hpp"
#include "lidar_processor.hpp"
#include "sl_lidar_cmd.h"
#include "sl_lidar_driver.h"
#include "sl_types.h"

namespace {

constexpr int MAP_WIDTH = 800;
constexpr int MAP_HEIGHT = 800;
constexpr float SCALE_PX_PER_M = 300.0f;
constexpr float PI = 3.14159265358979323846f;

const cv::Point2f ORIGIN(static_cast<float>(MAP_WIDTH) * 0.5f,
	static_cast<float>(MAP_HEIGHT) * 0.5f);

cv::Point world_to_pixel(const cv::Point2f &point_m) {
	return {
		static_cast<int>(std::lround(ORIGIN.x + point_m.x * SCALE_PX_PER_M)),
		static_cast<int>(std::lround(ORIGIN.y - point_m.y * SCALE_PX_PER_M))};
}

bool is_inside_map(const cv::Point &pixel) {
	return pixel.x >= 0 && pixel.x < MAP_WIDTH && pixel.y >= 0 &&
		pixel.y < MAP_HEIGHT;
}

void draw_corner(
	cv::Mat &map, const std::optional<lidar::CornerEstimate> &corner) {

	if (!corner.has_value()) {
		return;
	}

	const cv::Point pixel = world_to_pixel(corner->position);

	// Big circle
	cv::circle(map, pixel, 10, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);

	// X marker
	cv::line(map, pixel + cv::Point(-7, -7), pixel + cv::Point(7, 7),
		cv::Scalar(0, 165, 255), 2, cv::LINE_AA);

	cv::line(map, pixel + cv::Point(-7, 7), pixel + cv::Point(7, -7),
		cv::Scalar(0, 165, 255), 2, cv::LINE_AA);

	// Label
	cv::putText(map, "CORNER", pixel + cv::Point(12, -12),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
}

// Must use the same coordinate convention as LidarProcessor::polar2cartesian().
//
// Robot frame:
//   +X = right
//   +Y = front
//
// The LiDAR is mounted rotated 180 degrees, therefore the minus signs.
cv::Point2f raw_to_cartesian(const LidarPoint &point) {
	const float rad = point.angle_deg * PI / 180.0f;

	return {
		-point.distance_m * std::sin(rad), -point.distance_m * std::cos(rad)};
}

std::string fixed(float value, int precision) {
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(precision) << value;
	return stream.str();
}

void draw_raw_points(cv::Mat &map, const TimedLidarData &scan) {

	for (const auto &point : scan.points) {
		if (point.distance_m <= 0.0f) {
			continue;
		}

		const cv::Point pixel = world_to_pixel(raw_to_cartesian(point));

		if (!is_inside_map(pixel)) {
			continue;
		}

		cv::circle(map, pixel, 1, cv::Scalar(90, 90, 90), -1);
	}
}

void draw_line_segments(
	cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	for (const auto &segment : processed.line_segments) {
		cv::line(map, world_to_pixel(segment.start),
			world_to_pixel(segment.end), cv::Scalar(180, 180, 180), 1,
			cv::LINE_AA);
	}
}

void draw_wall(cv::Mat &map, const std::optional<lidar::LineSegment> &wall) {

	if (!wall.has_value()) {
		return;
	}

	cv::line(map, world_to_pixel(wall->start), world_to_pixel(wall->end),
		cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
}

void draw_walls(cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	draw_wall(map, processed.walls.left);
	draw_wall(map, processed.walls.right);
	draw_wall(map, processed.walls.front);
}

void draw_obstacles(cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	for (const auto &obstacle : processed.obstacles) {
		const cv::Point2f start = obstacle.start();
		const cv::Point2f end = obstacle.end();
		const cv::Point2f center = obstacle.center;

		cv::line(map, world_to_pixel(start), world_to_pixel(end),
			cv::Scalar(0, 0, 255), 4, cv::LINE_AA);

		cv::circle(map, world_to_pixel(center), 4, cv::Scalar(0, 255, 255), -1);

		const float distance_m = obstacle.distance_m();
		const float bearing_deg = obstacle.bearing_rad() * 180.0f / PI;

		const std::string label =
			"D:" + fixed(distance_m, 2) + " B:" + fixed(bearing_deg, 1);

		cv::putText(map, label, world_to_pixel(center) + cv::Point(8, -8),
			cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1,
			cv::LINE_AA);
	}
}

void draw_parking_wall(
	cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	if (!processed.parking_wall.has_value()) {
		return;
	}

	const auto &wall = *processed.parking_wall;

	cv::line(map, world_to_pixel(wall.start), world_to_pixel(wall.end),
		cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
}

void draw_robot_frame(cv::Mat &map) {
	const cv::Point origin_pixel(
		static_cast<int>(ORIGIN.x), static_cast<int>(ORIGIN.y));

	// LiDAR origin
	cv::circle(map, origin_pixel, 5, cv::Scalar(255, 0, 0), -1);

	// +Y / front
	cv::line(map, origin_pixel, origin_pixel + cv::Point(0, -30),
		cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

	cv::putText(map, "FRONT", origin_pixel + cv::Point(8, -32),
		cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1,
		cv::LINE_AA);
}

int count_walls(const lidar::ProcessedLidarData &processed) {
	int count = 0;

	count += processed.walls.left.has_value() ? 1 : 0;
	count += processed.walls.right.has_value() ? 1 : 0;
	count += processed.walls.front.has_value() ? 1 : 0;

	return count;
}

void draw_debug_info(cv::Mat &map, const TimedLidarData &scan,
	const lidar::ProcessedLidarData &processed, std::int64_t process_us,
	std::uint64_t frame_diff_us) {

	const std::string info = "Points: " + std::to_string(scan.points.size()) +
		"  Segments: " + std::to_string(processed.line_segments.size()) +
		"  Walls: " + std::to_string(count_walls(processed)) +
		"  Obstacles: " + std::to_string(processed.obstacles.size());

	const std::string timing =
		"Process: " + fixed(static_cast<float>(process_us) / 1000.0f, 2) +
		" ms  Frame: " + fixed(static_cast<float>(frame_diff_us) / 1000.0f, 2) +
		" ms";

	cv::putText(map, info, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.5,
		cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

	cv::putText(map, timing, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5,
		cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

} // namespace

int main() {

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

	cv::Mat debug_map(MAP_HEIGHT, MAP_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

	std::uint64_t previous_timestamp_us = 0;

	while (true) {
		TimedLidarData scan;

		if (!lidar.wait_for_data(scan)) {
			std::cerr << "Failed to get LiDAR data\n";
			break;
		}

		if (scan.points.empty()) {
			continue;
		}

		const auto process_start = std::chrono::steady_clock::now();

		const lidar::ProcessedLidarData processed =
			lidar_processor.process(scan);

		const auto corner = processed.corner;

		const auto process_end = std::chrono::steady_clock::now();

		const auto process_us =
			std::chrono::duration_cast<std::chrono::microseconds>(
				process_end - process_start)
				.count();

		std::uint64_t frame_diff_us = 0;

		if (previous_timestamp_us != 0) {
			frame_diff_us = scan.timestamp_us - previous_timestamp_us;
		}

		previous_timestamp_us = scan.timestamp_us;

		debug_map.setTo(cv::Scalar(0, 0, 0));

		// Draw from low-level data to high-level interpretation.
		draw_raw_points(debug_map, scan);
		draw_line_segments(debug_map, processed);
		draw_walls(debug_map, processed);
		// draw_corner(debug_map, corner);
		// draw_obstacles(debug_map, processed);
		// draw_parking_wall(debug_map, processed);
		draw_robot_frame(debug_map);
		draw_debug_info(debug_map, scan, processed, process_us, frame_diff_us);

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
