#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

#include "init_direction.hpp"
#include "lidar_module.hpp"
#include "lidar_processor.hpp"
#include "otos.hpp"

namespace {

// -----------------------------------------------------------------------------
// Map config
// -----------------------------------------------------------------------------

constexpr int MAP_WIDTH = 900;
constexpr int MAP_HEIGHT = 900;

constexpr float SCALE_PX_PER_M = 300.0f;
constexpr float PI = 3.14159265358979323846f;

const cv::Point2f ORIGIN(static_cast<float>(MAP_WIDTH) * 0.5f,
	static_cast<float>(MAP_HEIGHT) * 0.65f);

// -----------------------------------------------------------------------------
// Basic utilities
// -----------------------------------------------------------------------------

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

float normalize_angle(float angle_rad) {

	return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
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
// Raw LiDAR conversion
// -----------------------------------------------------------------------------

cv::Point2f raw_to_cartesian(const LidarPoint &point) {

	const float rad = point.angle_deg * PI / 180.0f;

	// Robot frame:
	//
	// +X = RIGHT
	// +Y = FRONT
	//
	// LiDAR mounted 180 degrees.

	return {-point.distance_m * std::sin(rad),

		-point.distance_m * std::cos(rad)};
}

// -----------------------------------------------------------------------------
// Grid
// -----------------------------------------------------------------------------

void draw_grid(cv::Mat &map) {

	constexpr float GRID_SPACING_M = 0.25f;

	const int spacing_px = static_cast<int>(GRID_SPACING_M * SCALE_PX_PER_M);

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

// -----------------------------------------------------------------------------
// Raw LiDAR points
// -----------------------------------------------------------------------------

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
// Fitted / merged segments
// -----------------------------------------------------------------------------

void draw_line_segments(
	cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	for (const auto &segment : processed.line_segments) {

		cv::line(map, world_to_pixel(segment.start),
			world_to_pixel(segment.end), cv::Scalar(140, 140, 140), 1,
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
		label + " " + fixed(wall->perpendicular_distance(), 2) + "m";

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
// Side-wall forward endpoints
// -----------------------------------------------------------------------------

cv::Point2f forward_endpoint(const lidar::LineSegment &segment) {

	return segment.start.y > segment.end.y ? segment.start : segment.end;
}

void draw_forward_endpoint(cv::Mat &map,
	const std::optional<lidar::LineSegment> &wall, const cv::Scalar &color) {

	if (!wall.has_value()) {
		return;
	}

	const cv::Point pixel = world_to_pixel(forward_endpoint(*wall));

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
// Obstacles
// -----------------------------------------------------------------------------

void draw_obstacles(cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	for (const auto &obstacle : processed.obstacles) {

		const cv::Point2f start = obstacle.start();

		const cv::Point2f end = obstacle.end();

		const cv::Point2f center = obstacle.center;

		cv::line(map, world_to_pixel(start), world_to_pixel(end),
			cv::Scalar(0, 0, 255), 4, cv::LINE_AA);

		cv::circle(map, world_to_pixel(center), 5, cv::Scalar(0, 255, 255), -1);

		const float distance_m = obstacle.distance_m();

		const float bearing_deg = obstacle.bearing_rad() * 180.0f / PI;

		const std::string label =
			"OBS D:" + fixed(distance_m, 2) + " B:" + fixed(bearing_deg, 1);

		cv::putText(map, label, world_to_pixel(center) + cv::Point(8, -8),
			cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1,
			cv::LINE_AA);
	}
}

// -----------------------------------------------------------------------------
// Parking wall
// -----------------------------------------------------------------------------

void draw_parking_wall(
	cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	if (!processed.parking_wall.has_value()) {
		return;
	}

	const auto &wall = *processed.parking_wall;

	cv::line(map, world_to_pixel(wall.start), world_to_pixel(wall.end),
		cv::Scalar(255, 0, 255), 3, cv::LINE_AA);

	const cv::Point2f center = (wall.start + wall.end) * 0.5f;

	cv::putText(map, "PARKING", world_to_pixel(center) + cv::Point(8, -8),
		cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 0, 255), 1,
		cv::LINE_AA);
}

// -----------------------------------------------------------------------------
// Robot frame
// -----------------------------------------------------------------------------

void draw_robot(cv::Mat &map) {

	const cv::Point origin(
		static_cast<int>(ORIGIN.x), static_cast<int>(ORIGIN.y));

	// Robot / LiDAR origin
	cv::circle(map, origin, 7, cv::Scalar(255, 0, 255), -1);

	// +Y FRONT
	cv::arrowedLine(map, origin, origin + cv::Point(0, -60),
		cv::Scalar(255, 255, 255), 2, cv::LINE_AA, 0, 0.2);

	cv::putText(map, "+Y FRONT", origin + cv::Point(10, -65),
		cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1,
		cv::LINE_AA);

	// +X RIGHT
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

	cv::putText(map, text, cv::Point(20, 100), cv::FONT_HERSHEY_SIMPLEX, 0.7,
		color, 2, cv::LINE_AA);

	if (!direction.has_value()) {
		return;
	}

	const cv::Point center(MAP_WIDTH - 100, 90);

	constexpr int radius = 35;

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
// Debug UI
// -----------------------------------------------------------------------------

int count_walls(const lidar::ProcessedLidarData &processed) {

	int count = 0;

	if (processed.walls.left) {
		++count;
	}

	if (processed.walls.right) {
		++count;
	}

	if (processed.walls.front) {
		++count;
	}

	return count;
}

void draw_debug_info(cv::Mat &map, const TimedLidarData &scan,
	const lidar::ProcessedLidarData &processed, std::int64_t process_us,
	std::uint64_t frame_diff_us, float heading_rad, float heading_error_rad,
	const std::optional<DrivingDirection> &direction) {

	const std::string line1 = "Points: " + std::to_string(scan.points.size()) +
		"  Segments: " + std::to_string(processed.line_segments.size()) +
		"  Walls: " + std::to_string(count_walls(processed)) +
		"  Obstacles: " + std::to_string(processed.obstacles.size());

	const std::string line2 =
		"Process: " + fixed(static_cast<float>(process_us) / 1000.0f, 2) +
		" ms  Frame: " + fixed(static_cast<float>(frame_diff_us) / 1000.0f, 2) +
		" ms";

	const float heading_deg = heading_rad * 180.0f / PI;

	const float error_deg = heading_error_rad * 180.0f / PI;

	const std::string line3 = "OTOS H: " + fixed(heading_deg, 1) +
		" deg  ERR: " + fixed(error_deg, 1) + " deg";

	const std::string line4 = direction.has_value()
		? "DIR STATUS: LOCKED"
		: "DIR STATUS: SEARCHING...";

	cv::putText(map, line1, cv::Point(20, 25), cv::FONT_HERSHEY_SIMPLEX, 0.5,
		cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

	cv::putText(map, line2, cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5,
		cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

	cv::putText(map, line3, cv::Point(20, 75), cv::FONT_HERSHEY_SIMPLEX, 0.5,
		cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

	cv::putText(map, line4, cv::Point(20, 125), cv::FONT_HERSHEY_SIMPLEX, 0.5,
		direction.has_value() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255),
		1, cv::LINE_AA);
}

} // namespace

// =============================================================================
// MAIN
// =============================================================================

int main() {

	// -------------------------------------------------------------------------
	// Modules
	// -------------------------------------------------------------------------

	lidar::LidarModule lidar("/dev/ttyAMA0", 1000000);

	lidar::LidarProcessor lidar_processor;

	otos::OTOS otos;

	// -------------------------------------------------------------------------
	// Initial direction estimator
	// -------------------------------------------------------------------------

	navigation::InitialDirectionConfig direction_config;

	direction_config.max_collinear_offset_m = 0.04f;

	direction_config.max_continuation_gap_m = 0.20f;

	direction_config.max_perpendicular_error_rad = 15.0f * PI / 180.0f;

	direction_config.max_connection_gap_m = 0.22f;

	direction_config.min_candidate_length_m = 0.15f;

	direction_config.frame_min_score = 6.0f;

	direction_config.frame_score_margin = 1.5f;

	direction_config.score_decay = 0.7f;

	direction_config.required_confirm_frames = 3;

	navigation::InitialDirectionEstimator direction_estimator(direction_config);

	std::optional<DrivingDirection> latched_direction;

	// -------------------------------------------------------------------------
	// LiDAR initialization
	// -------------------------------------------------------------------------

	std::cout << "Initializing LiDAR...\n";

	if (!lidar.initialize()) {

		std::cerr << "LiDAR initialize failed\n";

		return 1;
	}

	if (!lidar.start()) {

		std::cerr << "LiDAR start failed\n";

		return 1;
	}

	std::cout << "LiDAR started\n";

	// -------------------------------------------------------------------------
	// OTOS initialization
	// -------------------------------------------------------------------------

	std::cout << "Initializing OTOS...\n";

	if (!otos.initialize(1)) {

		std::cerr << "OTOS initialize failed\n";

		lidar.stop();

		return 1;
	}

	otos.setLinearUnit(kSfeOtosLinearUnitMeters);

	otos.setAngularUnit(kSfeOtosAngularUnitRadians);

	otos.resetTracking();

	std::cout << "OTOS connected\n";

	// -------------------------------------------------------------------------
	// Debug map
	// -------------------------------------------------------------------------

	cv::Mat debug_map(MAP_HEIGHT, MAP_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

	// -------------------------------------------------------------------------
	// Runtime state
	// -------------------------------------------------------------------------

	std::uint64_t previous_timestamp_us = 0;

	float reference_heading_rad = 0.0f;

	bool heading_initialized = false;

	std::cout << "\nLiDAR + Navigation Debug Started\n"
			  << "R = reset direction\n"
			  << "H = reset heading reference\n"
			  << "Q / ESC = quit\n\n";

	// -------------------------------------------------------------------------
	// Main loop
	// -------------------------------------------------------------------------

	while (true) {

		TimedLidarData scan;

		sfe_otos_pose2d_t pos;
		sfe_otos_pose2d_t vel;
		sfe_otos_pose2d_t acc;

		// -----------------------------------------------------------------
		// LiDAR
		// -----------------------------------------------------------------

		if (!lidar.wait_for_data(scan)) {

			std::cerr << "Failed to get LiDAR data\n";

			break;
		}

		if (scan.points.empty()) {
			continue;
		}

		// -----------------------------------------------------------------
		// OTOS
		// -----------------------------------------------------------------

		const sfTkError_t otos_error = otos.getPosVelAcc(pos, vel, acc);

		if (otos_error != ksfTkErrOk) {

			std::cerr << "OTOS read error: " << static_cast<int>(otos_error)
					  << '\n';

			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			continue;
		}

		// OTOS is configured in radians.
		const float heading_rad = pos.h;

		if (!heading_initialized) {

			reference_heading_rad = heading_rad;

			heading_initialized = true;

			std::cout << "Heading reference: "
					  << reference_heading_rad * 180.0f / PI << " deg\n";
		}

		const float heading_error_rad =
			normalize_angle(heading_rad - reference_heading_rad);

		// -----------------------------------------------------------------
		// LiDAR processing
		// -----------------------------------------------------------------

		const auto process_start = std::chrono::steady_clock::now();

		const lidar::ProcessedLidarData processed =
			lidar_processor.process(scan, heading_error_rad);

		// -----------------------------------------------------------------
		// Initial direction
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
		// Frame timing
		// -----------------------------------------------------------------

		std::uint64_t frame_diff_us = 0;

		if (previous_timestamp_us != 0) {

			frame_diff_us = scan.timestamp_us - previous_timestamp_us;
		}

		previous_timestamp_us = scan.timestamp_us;

		// -----------------------------------------------------------------
		// Draw
		// -----------------------------------------------------------------

		debug_map.setTo(cv::Scalar(0, 0, 0));

		draw_grid(debug_map);

		draw_raw_points(debug_map, scan);

		draw_line_segments(debug_map, processed);

		draw_walls(debug_map, processed);

		draw_side_endpoints(debug_map, processed);

		draw_obstacles(debug_map, processed);

		draw_parking_wall(debug_map, processed);

		draw_robot(debug_map);

		draw_direction(debug_map, latched_direction);

		draw_debug_info(debug_map, scan, processed, process_us, frame_diff_us,
			heading_rad, heading_error_rad, latched_direction);

		cv::imshow("LiDAR + Navigation Debug", debug_map);

		// -----------------------------------------------------------------
		// Keyboard
		// -----------------------------------------------------------------

		const int key = cv::waitKey(1);

		if (key == 'q' || key == 'Q' || key == 27) {

			break;
		}

		// Reset direction estimator
		if (key == 'r' || key == 'R') {

			latched_direction.reset();

			direction_estimator.reset();

			std::cout << "Direction reset\n";
		}

		// Use current heading as new straight reference
		if (key == 'h' || key == 'H') {

			reference_heading_rad = heading_rad;

			std::cout << "Heading reference reset: "
					  << reference_heading_rad * 180.0f / PI << " deg\n";
		}
	}

	// -------------------------------------------------------------------------
	// Shutdown
	// -------------------------------------------------------------------------

	lidar.stop();

	cv::destroyAllWindows();

	return 0;
}