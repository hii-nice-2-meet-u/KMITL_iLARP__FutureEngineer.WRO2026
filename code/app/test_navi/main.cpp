#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>

#include "initial_direction_estimator.hpp"
#include "lidar_module.hpp"
#include "lidar_processor.hpp"

namespace {

constexpr int MAP_WIDTH = 900;
constexpr int MAP_HEIGHT = 900;

constexpr float SCALE_PX_PER_M = 300.0f;
constexpr float PI = static_cast<float>(M_PI);

const cv::Point2f ORIGIN(static_cast<float>(MAP_WIDTH) * 0.5f,
	static_cast<float>(MAP_HEIGHT) * 0.65f);

cv::Point world_to_pixel(const cv::Point2f &point_m) {

	return {
		static_cast<int>(std::lround(ORIGIN.x + point_m.x * SCALE_PX_PER_M)),

		static_cast<int>(std::lround(ORIGIN.y - point_m.y * SCALE_PX_PER_M))};
}

bool is_inside_map(const cv::Point &pixel) {

	return pixel.x >= 0 && pixel.x < MAP_WIDTH && pixel.y >= 0 &&
		pixel.y < MAP_HEIGHT;
}

std::string fixed(float value, int precision = 2) {

	std::ostringstream stream;

	stream << std::fixed << std::setprecision(precision) << value;

	return stream.str();
}

std::string direction_to_string(
	const std::optional<DrivingDirection> &direction) {

	if (!direction.has_value()) {
		return "UNKNOWN";
	}

	switch (*direction) {

	case DrivingDirection::CLOCKWISE:
		return "CLOCKWISE";

	case DrivingDirection::COUNTER_CLOCKWISE:
		return "COUNTER CLOCKWISE";
	}

	return "UNKNOWN";
}

// -----------------------------------------------------------------------------
// Raw LiDAR
// -----------------------------------------------------------------------------

cv::Point2f raw_to_cartesian(const LidarPoint &point) {

	const float rad = point.angle_deg * PI / 180.0f;

	// Robot:
	// +X = right
	// +Y = front
	//
	// LiDAR mounted 180 deg

	return {
		-point.distance_m * std::sin(rad), -point.distance_m * std::cos(rad)};
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

		cv::circle(map, pixel, 1, cv::Scalar(70, 70, 70), -1);
	}
}

// -----------------------------------------------------------------------------
// All fitted segments
// -----------------------------------------------------------------------------

void draw_segments(cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	for (const auto &segment : processed.line_segments) {

		cv::line(map, world_to_pixel(segment.start),
			world_to_pixel(segment.end), cv::Scalar(120, 120, 120), 1,
			cv::LINE_AA);
	}
}

// -----------------------------------------------------------------------------
// Resolved walls
// -----------------------------------------------------------------------------

void draw_wall(cv::Mat &map, const std::optional<lidar::LineSegment> &wall,
	const cv::Scalar &color, const std::string &label) {

	if (!wall.has_value()) {
		return;
	}

	const cv::Point start = world_to_pixel(wall->start);

	const cv::Point end = world_to_pixel(wall->end);

	cv::line(map, start, end, color, 4, cv::LINE_AA);

	const cv::Point2f center_m = (wall->start + wall->end) * 0.5f;

	const cv::Point center = world_to_pixel(center_m);

	cv::circle(map, center, 5, color, -1, cv::LINE_AA);

	const std::string text =
		label + " " + fixed(wall->perpendicular_distance()) + "m";

	cv::putText(map, text, center + cv::Point(8, -8), cv::FONT_HERSHEY_SIMPLEX,
		0.45, color, 1, cv::LINE_AA);
}

void draw_walls(cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	// LEFT = cyan
	draw_wall(map, processed.walls.left, cv::Scalar(255, 255, 0), "LEFT");

	// RIGHT = green
	draw_wall(map, processed.walls.right, cv::Scalar(0, 255, 0), "RIGHT");

	// FRONT = yellow
	draw_wall(map, processed.walls.front, cv::Scalar(0, 255, 255), "FRONT");
}

// -----------------------------------------------------------------------------
// Draw forward endpoints
// -----------------------------------------------------------------------------

cv::Point2f forward_endpoint(const lidar::LineSegment &segment) {

	return segment.start.y > segment.end.y ? segment.start : segment.end;
}

void draw_forward_endpoint(cv::Mat &map,
	const std::optional<lidar::LineSegment> &wall, const cv::Scalar &color) {

	if (!wall.has_value()) {
		return;
	}

	const cv::Point2f point = forward_endpoint(*wall);

	const cv::Point pixel = world_to_pixel(point);

	cv::circle(map, pixel, 9, color, 2, cv::LINE_AA);

	cv::line(map, pixel + cv::Point(-6, -6), pixel + cv::Point(6, 6), color, 2,
		cv::LINE_AA);

	cv::line(map, pixel + cv::Point(-6, 6), pixel + cv::Point(6, -6), color, 2,
		cv::LINE_AA);
}

void draw_side_endpoints(
	cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	draw_forward_endpoint(map, processed.walls.left, cv::Scalar(255, 255, 0));

	draw_forward_endpoint(map, processed.walls.right, cv::Scalar(0, 255, 0));
}

// -----------------------------------------------------------------------------
// Robot
// -----------------------------------------------------------------------------

void draw_robot(cv::Mat &map) {

	const cv::Point origin{
		static_cast<int>(ORIGIN.x), static_cast<int>(ORIGIN.y)};

	// robot center
	cv::circle(map, origin, 7, cv::Scalar(255, 0, 255), -1);

	// Front arrow
	cv::arrowedLine(map, origin, origin + cv::Point(0, -60),
		cv::Scalar(255, 255, 255), 2, cv::LINE_AA, 0, 0.2);

	cv::putText(map, "+Y FRONT", origin + cv::Point(10, -65),
		cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1,
		cv::LINE_AA);

	// +X
	cv::arrowedLine(map, origin, origin + cv::Point(50, 0),
		cv::Scalar(255, 255, 255), 1, cv::LINE_AA, 0, 0.2);

	cv::putText(map, "+X RIGHT", origin + cv::Point(55, 5),
		cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1,
		cv::LINE_AA);
}

// -----------------------------------------------------------------------------
// Direction UI
// -----------------------------------------------------------------------------

void draw_direction(
	cv::Mat &map, const std::optional<DrivingDirection> &direction) {

	const std::string text = "DIRECTION: " + direction_to_string(direction);

	cv::Scalar color;

	if (!direction.has_value()) {

		color = cv::Scalar(150, 150, 150);

	} else if (*direction == DrivingDirection::CLOCKWISE) {

		color = cv::Scalar(0, 255, 0);

	} else {

		color = cv::Scalar(255, 255, 0);
	}

	cv::putText(map, text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8,
		color, 2, cv::LINE_AA);

	// ---------------------------------------------------------
	// Draw turning arrow
	// ---------------------------------------------------------

	if (!direction.has_value()) {
		return;
	}

	const cv::Point center(MAP_WIDTH - 100, 90);

	const int radius = 35;

	if (*direction == DrivingDirection::CLOCKWISE) {

		cv::ellipse(map, center, cv::Size(radius, radius), 0, -60, 240, color,
			3, cv::LINE_AA);

		cv::arrowedLine(map, center + cv::Point(-30, -18),
			center + cv::Point(-25, -32), color, 3);

	} else {

		cv::ellipse(map, center, cv::Size(radius, radius), 0, -240, 60, color,
			3, cv::LINE_AA);

		cv::arrowedLine(map, center + cv::Point(30, -18),
			center + cv::Point(25, -32), color, 3);
	}
}

// -----------------------------------------------------------------------------
// Debug information
// -----------------------------------------------------------------------------

int count_walls(const lidar::ProcessedLidarData &processed) {

	int count = 0;

	if (processed.walls.left) ++count;

	if (processed.walls.right) ++count;

	if (processed.walls.front) ++count;

	return count;
}

void draw_debug_info(cv::Mat &map, const TimedLidarData &scan,
	const lidar::ProcessedLidarData &processed, std::int64_t process_us,
	bool locked) {

	const std::string line1 = "Points: " + std::to_string(scan.points.size()) +
		"  Segments: " + std::to_string(processed.line_segments.size()) +
		"  Walls: " + std::to_string(count_walls(processed));

	cv::putText(map, line1, cv::Point(20, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5,
		cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

	const std::string line2 =
		"Process: " + fixed(static_cast<float>(process_us) / 1000.0f) + " ms";

	cv::putText(map, line2, cv::Point(20, 95), cv::FONT_HERSHEY_SIMPLEX, 0.5,
		cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

	const std::string status = locked ? "LOCKED" : "SEARCHING...";

	cv::putText(map, status, cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.55,
		locked ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), 2,
		cv::LINE_AA);
}

// -----------------------------------------------------------------------------
// Grid
// -----------------------------------------------------------------------------

void draw_grid(cv::Mat &map) {

	const float spacing_m = 0.25f;

	const int spacing_px = static_cast<int>(spacing_m * SCALE_PX_PER_M);

	for (int x = static_cast<int>(ORIGIN.x); x < MAP_WIDTH; x += spacing_px) {

		cv::line(map, cv::Point(x, 0), cv::Point(x, MAP_HEIGHT),
			cv::Scalar(25, 25, 25), 1);
	}

	for (int x = static_cast<int>(ORIGIN.x); x >= 0; x -= spacing_px) {

		cv::line(map, cv::Point(x, 0), cv::Point(x, MAP_HEIGHT),
			cv::Scalar(25, 25, 25), 1);
	}

	for (int y = static_cast<int>(ORIGIN.y); y < MAP_HEIGHT; y += spacing_px) {

		cv::line(map, cv::Point(0, y), cv::Point(MAP_WIDTH, y),
			cv::Scalar(25, 25, 25), 1);
	}

	for (int y = static_cast<int>(ORIGIN.y); y >= 0; y -= spacing_px) {

		cv::line(map, cv::Point(0, y), cv::Point(MAP_WIDTH, y),
			cv::Scalar(25, 25, 25), 1);
	}
}

} // namespace

int main() {

	// -------------------------------------------------------------------------
	// LiDAR
	// -------------------------------------------------------------------------

	lidar::LidarModule lidar("/dev/ttyAMA0", 1000000);

	lidar::LidarProcessor lidar_processor;

	// -------------------------------------------------------------------------
	// Direction estimator
	// -------------------------------------------------------------------------

	navigation::InitialDirectionConfig config;

	config.max_collinear_offset_m = 0.04f;

	config.max_continuation_gap_m = 0.20f;

	config.max_perpendicular_error_rad = 15.0f * PI / 180.0f;

	config.max_connection_gap_m = 0.22f;

	config.min_candidate_length_m = 0.15f;

	config.frame_min_score = 6.0f;

	config.frame_score_margin = 1.5f;

	config.score_decay = 0.7f;

	config.required_confirm_frames = 3;

	navigation::InitialDirectionEstimator direction_estimator(config);

	// -------------------------------------------------------------------------
	// Initialize
	// -------------------------------------------------------------------------

	if (!lidar.initialize()) {

		std::cerr << "LiDAR initialize failed\n";

		return 1;
	}

	if (!lidar.start()) {

		std::cerr << "LiDAR start failed\n";

		return 1;
	}

	cv::Mat map(MAP_HEIGHT, MAP_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

	std::optional<DrivingDirection> latched_direction;

	std::cout << "Navigation visualization started\n";

	std::cout << "R = reset direction\n"
			  << "Q / ESC = quit\n";

	// -------------------------------------------------------------------------
	// Main loop
	// -------------------------------------------------------------------------

	while (true) {

		TimedLidarData scan;

		if (!lidar.wait_for_data(scan)) {
			break;
		}

		if (scan.points.empty()) {
			continue;
		}

		const auto process_start = std::chrono::steady_clock::now();

		const lidar::ProcessedLidarData processed =
			lidar_processor.process(scan);

		// -----------------------------------------------------------------
		// Direction estimation
		// -----------------------------------------------------------------

		if (!latched_direction.has_value()) {

			const auto direction = direction_estimator.update(processed);

			if (direction.has_value()) {

				latched_direction = *direction;

				std::cout << "DIRECTION LOCKED: "
						  << direction_to_string(latched_direction) << '\n';
			}
		}

		const auto process_end = std::chrono::steady_clock::now();

		const auto process_us =
			std::chrono::duration_cast<std::chrono::microseconds>(
				process_end - process_start)
				.count();

		// -----------------------------------------------------------------
		// Draw
		// -----------------------------------------------------------------

		map.setTo(cv::Scalar(0, 0, 0));

		draw_grid(map);

		draw_raw_points(map, scan);

		draw_segments(map, processed);

		draw_walls(map, processed);

		draw_side_endpoints(map, processed);

		draw_robot(map);

		draw_direction(map, latched_direction);

		draw_debug_info(
			map, scan, processed, process_us, latched_direction.has_value());

		cv::imshow("Navigation LiDAR Debug", map);

		// -----------------------------------------------------------------
		// Keyboard
		// -----------------------------------------------------------------

		const int key = cv::waitKey(1);

		if (key == 'q' || key == 'Q' || key == 27) {

			break;
		}

		if (key == 'r' || key == 'R') {

			latched_direction.reset();

			direction_estimator.reset();

			std::cout << "Direction reset\n";
		}
	}

	lidar.stop();

	cv::destroyAllWindows();

	return 0;
}