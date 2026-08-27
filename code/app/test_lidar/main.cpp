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

#include "lidar_module.hpp"
#include "lidar_processor.hpp"
#include "navigation_controller.hpp"
#include "otos.hpp"

namespace {

// =============================================================================
// CONFIG
// =============================================================================

constexpr int MAP_WIDTH = 900;
constexpr int MAP_HEIGHT = 900;

constexpr float SCALE_PX_PER_M = 300.0f;

constexpr float PI = 3.14159265358979323846f;

constexpr float RAD_TO_DEG = 180.0f / PI;

// Servo calibration
constexpr int SERVO_MIN_US = 1000;
constexpr int SERVO_CENTER_US = 1550;
constexpr int SERVO_MAX_US = 2100;

const cv::Point2f ORIGIN(static_cast<float>(MAP_WIDTH) * 0.5f,
	static_cast<float>(MAP_HEIGHT) * 0.65f);

// =============================================================================
// BASIC UTILITIES
// =============================================================================

float normalize_angle(float angle_rad) {

	return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

std::string fixed(float value, int precision = 2) {

	std::ostringstream stream;

	stream << std::fixed << std::setprecision(precision) << value;

	return stream.str();
}

cv::Point world_to_pixel(const cv::Point2f &point_m) {

	return {
		static_cast<int>(std::lround(ORIGIN.x + point_m.x * SCALE_PX_PER_M)),

		static_cast<int>(std::lround(ORIGIN.y - point_m.y * SCALE_PX_PER_M))};
}

bool is_inside_map(const cv::Point &pixel) {

	return pixel.x >= 0 && pixel.x < MAP_WIDTH && pixel.y >= 0 &&
		pixel.y < MAP_HEIGHT;
}

// =============================================================================
// STRING
// =============================================================================

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

std::string mode_to_string(navigation::NavigationMode mode) {

	switch (mode) {

	case navigation::NavigationMode::SEARCH_DIRECTION:
		return "SEARCH_DIRECTION";

	case navigation::NavigationMode::NORMAL:
		return "NORMAL";

	case navigation::NavigationMode::TURNING:
		return "TURNING";

	case navigation::NavigationMode::FINISHED:
		return "FINISHED";
	}

	return "UNKNOWN";
}

// =============================================================================
// SERVO DEBUG MAPPING
//
// This DOES NOT write to servo.
//
// Navigation output:
// steering < 0 = LEFT
// steering > 0 = RIGHT
//
// Servo:
// LEFT   = 1000 us
// CENTER = 1550 us
// RIGHT  = 2100 us
// =============================================================================

int steering_to_servo_us(float steering_rad, float max_steering_rad) {

	if (max_steering_rad <= 0.0f) {
		return SERVO_CENTER_US;
	}

	steering_rad =
		std::clamp(steering_rad, -max_steering_rad, max_steering_rad);

	if (steering_rad >= 0.0f) {

		const float ratio = steering_rad / max_steering_rad;

		return SERVO_CENTER_US +
			static_cast<int>(
				ratio * static_cast<float>(SERVO_MAX_US - SERVO_CENTER_US));
	}

	const float ratio = -steering_rad / max_steering_rad;

	return SERVO_CENTER_US -
		static_cast<int>(
			ratio * static_cast<float>(SERVO_CENTER_US - SERVO_MIN_US));
}

// =============================================================================
// RAW LIDAR
// =============================================================================

cv::Point2f raw_to_cartesian(const LidarPoint &point) {

	const float rad = point.angle_deg * PI / 180.0f;

	// Robot frame:
	//
	// +X = RIGHT
	// +Y = FRONT
	//
	// LiDAR mounted 180 deg.

	return {-point.distance_m * std::sin(rad),

		-point.distance_m * std::cos(rad)};
}

// =============================================================================
// GRID
// =============================================================================

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

// =============================================================================
// RAW POINTS
// =============================================================================

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

// =============================================================================
// SEGMENTS
// =============================================================================

void draw_line_segments(
	cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	for (const auto &segment : processed.line_segments) {

		cv::line(map, world_to_pixel(segment.start),
			world_to_pixel(segment.end), cv::Scalar(140, 140, 140), 1,
			cv::LINE_AA);
	}
}

// =============================================================================
// WALLS
// =============================================================================

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

	draw_wall(map, processed.walls.left, cv::Scalar(255, 255, 0), "LEFT");

	draw_wall(map, processed.walls.right, cv::Scalar(0, 255, 0), "RIGHT");

	draw_wall(map, processed.walls.front, cv::Scalar(0, 255, 255), "FRONT");
}

// =============================================================================
// OUTER WALL HIGHLIGHT
// =============================================================================

void draw_outer_wall(cv::Mat &map, const lidar::ProcessedLidarData &processed,
	const std::optional<DrivingDirection> &direction) {

	if (!direction.has_value()) {
		return;
	}

	const std::optional<lidar::LineSegment> *outer = nullptr;

	if (*direction == DrivingDirection::CLOCKWISE) {

		outer = &processed.walls.left;

	} else {

		outer = &processed.walls.right;
	}

	if (outer == nullptr || !outer->has_value()) {

		return;
	}

	cv::line(map, world_to_pixel((*outer)->start),
		world_to_pixel((*outer)->end), cv::Scalar(255, 0, 255), 7, cv::LINE_AA);

	const cv::Point2f center = ((*outer)->start + (*outer)->end) * 0.5f;

	cv::putText(map, "OUTER", world_to_pixel(center) + cv::Point(10, 15),
		cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 0, 255), 2,
		cv::LINE_AA);
}

// =============================================================================
// OBSTACLES
// =============================================================================

void draw_obstacles(cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	for (const auto &obstacle : processed.obstacles) {

		const cv::Point2f start = obstacle.start();

		const cv::Point2f end = obstacle.end();

		const cv::Point2f center = obstacle.center;

		cv::line(map, world_to_pixel(start), world_to_pixel(end),
			cv::Scalar(0, 0, 255), 4, cv::LINE_AA);

		cv::circle(map, world_to_pixel(center), 5, cv::Scalar(0, 255, 255), -1);

		const float distance_m = obstacle.distance_m();

		const float bearing_deg = obstacle.bearing_rad() * RAD_TO_DEG;

		const std::string label =
			"OBS D:" + fixed(distance_m, 2) + " B:" + fixed(bearing_deg, 1);

		cv::putText(map, label, world_to_pixel(center) + cv::Point(8, -8),
			cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1,
			cv::LINE_AA);
	}
}

// =============================================================================
// PARKING
// =============================================================================

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

// =============================================================================
// ROBOT
// =============================================================================

void draw_robot(cv::Mat &map) {

	const cv::Point origin(
		static_cast<int>(ORIGIN.x), static_cast<int>(ORIGIN.y));

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

// =============================================================================
// STEERING ARROW
// =============================================================================

void draw_steering(cv::Mat &map, float steering_rad) {

	const cv::Point origin(
		static_cast<int>(ORIGIN.x), static_cast<int>(ORIGIN.y));

	constexpr float LENGTH_PX = 90.0f;

	const int dx = static_cast<int>(std::sin(steering_rad) * LENGTH_PX);

	const int dy = static_cast<int>(-std::cos(steering_rad) * LENGTH_PX);

	const cv::Point end = origin + cv::Point(dx, dy);

	cv::arrowedLine(
		map, origin, end, cv::Scalar(0, 128, 255), 4, cv::LINE_AA, 0, 0.2);

	cv::putText(map, "STEER", end + cv::Point(8, 0), cv::FONT_HERSHEY_SIMPLEX,
		0.45, cv::Scalar(0, 128, 255), 2, cv::LINE_AA);
}

// =============================================================================
// COUNT WALLS
// =============================================================================

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

// =============================================================================
// NAVIGATION DEBUG PANEL
// =============================================================================

void draw_navigation_info(cv::Mat &map, const TimedLidarData &scan,
	const lidar::ProcessedLidarData &processed,
	const navigation::NavigationResult &nav_result,
	const navigation::NavigationState &nav_state, std::int64_t process_us,
	std::uint64_t frame_diff_us, float heading_rad,
	float wall_heading_error_rad, float speed_mps, float max_steering_rad) {

	const float steering_deg = nav_result.command.steering_rad * RAD_TO_DEG;

	const float heading_deg = heading_rad * RAD_TO_DEG;

	const float wall_error_deg = wall_heading_error_rad * RAD_TO_DEG;

	const float turn_error_deg =
		nav_result.debug.heading_error_rad * RAD_TO_DEG;

	const float wall_angle_deg = nav_result.debug.angle_error_rad * RAD_TO_DEG;

	const int servo_us =
		steering_to_servo_us(nav_result.command.steering_rad, max_steering_rad);

	const std::string line1 = "MODE: " + mode_to_string(nav_state.mode) +
		"  DIR: " + direction_to_string(nav_state.direction);

	const std::string line2 = "Points:" + std::to_string(scan.points.size()) +
		" Seg:" + std::to_string(processed.line_segments.size()) +
		" Walls:" + std::to_string(count_walls(processed)) +
		" Obs:" + std::to_string(processed.obstacles.size());

	const std::string line3 = "OTOS H:" + fixed(heading_deg, 1) +
		"deg  WallCorr:" + fixed(wall_error_deg, 1) + "deg";

	const std::string line4 = "Speed:" + fixed(speed_mps, 2) +
		" m/s  Target:" + fixed(nav_result.command.target_speed_mps, 2) +
		" m/s";

	const std::string line5 =
		"Outer:" + fixed(nav_result.debug.outer_distance_m, 3) +
		"m  CTE:" + fixed(nav_result.debug.distance_error_m, 3) + "m";

	const std::string line6 = "WallErr:" + fixed(wall_angle_deg, 1) +
		"deg  TurnErr:" + fixed(turn_error_deg, 1) + "deg";

	const std::string line7 = "Steer:" + fixed(steering_deg, 1) +
		"deg  Servo:" + std::to_string(servo_us) + "us";

	const std::string line8 = "Turn:" + std::to_string(nav_state.turn_count) +
		"/12  Lap:" + std::to_string(nav_state.lap) +
		"  Corner:" + std::to_string(nav_state.corner_index);

	const std::string line9 =
		"Process:" + fixed(static_cast<float>(process_us) / 1000.0f, 2) +
		"ms  Frame:" + fixed(static_cast<float>(frame_diff_us) / 1000.0f, 2) +
		"ms";

	const int x = 20;

	int y = 25;

	constexpr int DY = 24;

	auto draw_line = [&](const std::string &text, const cv::Scalar &color) {
		cv::putText(map, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.48,
			color, 1, cv::LINE_AA);

		y += DY;
	};

	draw_line(line1, cv::Scalar(0, 255, 255));

	draw_line(line2, cv::Scalar(255, 255, 255));

	draw_line(line3, cv::Scalar(255, 255, 255));

	draw_line(line4, cv::Scalar(0, 255, 0));

	draw_line(line5, cv::Scalar(255, 255, 255));

	draw_line(line6, cv::Scalar(255, 255, 255));

	draw_line(line7, cv::Scalar(0, 128, 255));

	draw_line(line8, cv::Scalar(255, 255, 255));

	draw_line(line9, cv::Scalar(150, 150, 150));
}

} // namespace

// =============================================================================
// MAIN
// =============================================================================

int main() {

	// =========================================================================
	// MODULES
	// =========================================================================

	lidar::LidarModule lidar("/dev/ttyAMA0", 1000000);

	lidar::LidarProcessor lidar_processor;

	otos::OTOS otos;

	// =========================================================================
	// NAVIGATION CONFIG
	// =========================================================================

	navigation::NavigationConfig nav_config;

	// Outer-wall virtual path.
	nav_config.target_outer_distance_m = 0.30f;

	// Stanley
	nav_config.stanley.k = 1.0f;

	nav_config.stanley.softening_speed_mps = 0.20f;

	nav_config.stanley.max_steering_rad = 30.0f * PI / 180.0f;

	// Search centering
	nav_config.search_center_kp = 0.8f;

	// Corner
	nav_config.approach_distance_m = 0.80f;

	nav_config.turn_trigger_distance_m = 0.50f;

	nav_config.turn_rearm_distance_m = 0.80f;

	// Turning
	nav_config.turn_heading_kp = 1.5f;

	nav_config.heading_tolerance_rad = 5.0f * PI / 180.0f;

	nav_config.heading_confirm_frames = 3;

	// Assumption:
	//
	// OTOS +heading = CCW
	nav_config.clockwise_turn_delta_rad = -PI * 0.5f;

	nav_config.counter_clockwise_turn_delta_rad = PI * 0.5f;

	// steering:
	// negative LEFT
	// positive RIGHT
	nav_config.heading_to_steering_sign = -1.0f;

	// =========================================================================
	// SAFE TEST SPEEDS
	//
	// No motor is controlled in this app yet.
	// These are only Navigation target values.
	// =========================================================================

	nav_config.search_speed_mps = 0.20f;

	nav_config.normal_speed_mps = 0.50f;

	nav_config.approach_speed_mps = 0.35f;

	nav_config.turning_speed_mps = 0.25f;

	nav_config.lost_wall_speed_mps = 0.20f;

	nav_config.total_turns = 12;

	// =========================================================================
	// INITIAL DIRECTION CONFIG
	// =========================================================================

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

	// =========================================================================
	// NAVIGATION CONTROLLER
	// =========================================================================

	navigation::NavigationController navigation(nav_config, direction_config);

	// =========================================================================
	// LIDAR INIT
	// =========================================================================

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

	// =========================================================================
	// OTOS INIT
	// =========================================================================

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

	// =========================================================================
	// DEBUG MAP
	// =========================================================================

	cv::Mat debug_map(MAP_HEIGHT, MAP_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

	// =========================================================================
	// RUNTIME
	// =========================================================================

	bool navigation_initialized = false;

	std::uint64_t previous_timestamp_us = 0;

	std::cout << "\n"
			  << "LiDAR + OTOS + Navigation + Stanley\n"
			  << "\n"
			  << "R = reset navigation\n"
			  << "Q / ESC = quit\n"
			  << "\n";

	// =========================================================================
	// MAIN LOOP
	// =========================================================================

	while (true) {

		TimedLidarData scan;

		sfe_otos_pose2d_t pos{};
		sfe_otos_pose2d_t vel{};
		sfe_otos_pose2d_t acc{};

		// =====================================================================
		// LIDAR FRAME
		// =====================================================================

		if (!lidar.wait_for_data(scan)) {

			std::cerr << "Failed to get LiDAR data\n";

			break;
		}

		if (scan.points.empty()) {
			continue;
		}

		// =====================================================================
		// OTOS
		// =====================================================================

		const sfTkError_t otos_error = otos.getPosVelAcc(pos, vel, acc);

		if (otos_error != ksfTkErrOk) {

			std::cerr << "OTOS read error: " << static_cast<int>(otos_error)
					  << '\n';

			continue;
		}

		const float heading_rad = pos.h;

		// Stanley only requires speed magnitude.
		// For Speed PID later, use proper longitudinal velocity.
		const float speed_mps = std::hypot(vel.x, vel.y);

		// =====================================================================
		// INITIALIZE NAVIGATION HEADING
		// =====================================================================

		if (!navigation_initialized) {

			navigation.reset(heading_rad);

			navigation_initialized = true;

			std::cout << "Navigation heading reference: "
					  << heading_rad * RAD_TO_DEG << " deg\n";
		}

		// =====================================================================
		// WALL HEADING CORRECTION
		//
		// IMPORTANT:
		//
		// Do NOT use global OTOS heading directly.
		//
		// Lidar wall resolver needs:
		//
		// current heading
		// -
		// current straight target heading
		//
		// NORMAL:
		// target = heading of current straight
		//
		// TURNING:
		// target was already changed by +/-90 deg.
		//
		// Navigation does not strongly depend on resolved walls while TURNING.
		// =====================================================================

		const float wall_heading_error_rad = normalize_angle(
			heading_rad - navigation.state().target_heading_rad);

		// =====================================================================
		// LIDAR PROCESS
		// =====================================================================

		const auto process_start = std::chrono::steady_clock::now();

		const lidar::ProcessedLidarData processed =
			lidar_processor.process(scan, wall_heading_error_rad);

		// =====================================================================
		// NAVIGATION
		//
		// Input:
		// LiDAR geometry
		// OTOS heading
		// OTOS speed
		//
		// Output:
		// target_speed_mps
		// steering_rad
		// =====================================================================

		const navigation::NavigationResult nav_result =
			navigation.update(processed, heading_rad, speed_mps);

		const auto process_end = std::chrono::steady_clock::now();

		const auto process_us =
			std::chrono::duration_cast<std::chrono::microseconds>(
				process_end - process_start)
				.count();

		// =====================================================================
		// FRAME TIME
		// =====================================================================

		std::uint64_t frame_diff_us = 0;

		if (previous_timestamp_us != 0) {

			frame_diff_us = scan.timestamp_us - previous_timestamp_us;
		}

		previous_timestamp_us = scan.timestamp_us;

		// =====================================================================
		// CONSOLE DEBUG
		// =====================================================================

		const auto &state = navigation.state();

		const int servo_us =
			steering_to_servo_us(nav_result.command.steering_rad,
				nav_config.stanley.max_steering_rad);

		std::cout << '\r' << "MODE=" << mode_to_string(state.mode)
				  << " DIR=" << direction_to_string(state.direction)
				  << " V=" << fixed(speed_mps, 2)
				  << " VT=" << fixed(nav_result.command.target_speed_mps, 2)
				  << " STEER="
				  << fixed(nav_result.command.steering_rad * RAD_TO_DEG, 1)
				  << "deg" << " SERVO=" << servo_us << "us"
				  << " TURN=" << state.turn_count << " LAP=" << state.lap
				  << "       " << std::flush;

		// =====================================================================
		// DRAW
		// =====================================================================

		debug_map.setTo(cv::Scalar(0, 0, 0));

		draw_grid(debug_map);

		draw_raw_points(debug_map, scan);

		draw_line_segments(debug_map, processed);

		draw_walls(debug_map, processed);

		draw_outer_wall(debug_map, processed, state.direction);

		draw_obstacles(debug_map, processed);

		draw_parking_wall(debug_map, processed);

		draw_robot(debug_map);

		draw_steering(debug_map, nav_result.command.steering_rad);

		draw_navigation_info(debug_map, scan, processed, nav_result, state,
			process_us, frame_diff_us, heading_rad, wall_heading_error_rad,
			speed_mps, nav_config.stanley.max_steering_rad);

		cv::imshow("LiDAR + OTOS + Navigation + Stanley", debug_map);

		// =====================================================================
		// KEYBOARD
		// =====================================================================

		const int key = cv::waitKey(1);

		if (key == 'q' || key == 'Q' || key == 27) {

			break;
		}

		// Reset complete navigation state:
		//
		// direction
		// turn count
		// lap
		// target heading
		if (key == 'r' || key == 'R') {

			navigation.reset(heading_rad);

			std::cout << "\nNavigation reset\n";
		}
	}

	// =========================================================================
	// SHUTDOWN
	// =========================================================================

	std::cout << "\nStopping...\n";

	lidar.stop();

	cv::destroyAllWindows();

	return 0;
}