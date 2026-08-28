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
#include "navigation_controller.hpp"
#include "otos.hpp"

namespace {

// =============================================================================
// CONFIG
// =============================================================================

constexpr int WORLD_VIEW_WIDTH = 920;
constexpr int PANEL_WIDTH = 500;
constexpr int MAP_WIDTH = WORLD_VIEW_WIDTH + PANEL_WIDTH;
constexpr int MAP_HEIGHT = 900;

constexpr float SCALE_PX_PER_M = 300.0f;

constexpr float PI = 3.14159265358979323846f;
constexpr float RAD_TO_DEG = 180.0f / PI;

const cv::Point2f ORIGIN(static_cast<float>(WORLD_VIEW_WIDTH) * 0.50f,
	static_cast<float>(MAP_HEIGHT) * 0.68f);

// =============================================================================
// UTIL
// =============================================================================

float normalize_angle(float angle_rad) {

	return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

std::string fixed(float value, int precision = 2) {

	std::ostringstream ss;

	ss << std::fixed << std::setprecision(precision) << value;

	return ss.str();
}

cv::Point world_to_pixel(const cv::Point2f &point_m) {

	return {
		static_cast<int>(std::lround(ORIGIN.x + point_m.x * SCALE_PX_PER_M)),

		static_cast<int>(std::lround(ORIGIN.y - point_m.y * SCALE_PX_PER_M))};
}

bool is_inside_world_view(const cv::Point &p) {

	return p.x >= 0 && p.x < WORLD_VIEW_WIDTH && p.y >= 0 && p.y < MAP_HEIGHT;
}

// =============================================================================
// STRING
// =============================================================================

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

std::string direction_to_string(
	const std::optional<DrivingDirection> &direction) {

	if (!direction.has_value()) {
		return "UNKNOWN";
	}

	if (*direction == DrivingDirection::CLOCKWISE) {

		return "CLOCKWISE";
	}

	return "COUNTER_CLOCKWISE";
}

std::string steering_to_string(float steering_rad) {

	constexpr float DEAD_BAND_RAD = 0.01f;

	if (steering_rad > DEAD_BAND_RAD) {

		return "RIGHT";
	}

	if (steering_rad < -DEAD_BAND_RAD) {

		return "LEFT";
	}

	return "CENTER";
}

std::string cte_to_string(float cte_m) {

	constexpr float DEAD_BAND_M = 0.005f;

	if (cte_m > DEAD_BAND_M) {

		return "RIGHT";
	}

	if (cte_m < -DEAD_BAND_M) {

		return "LEFT";
	}

	return "CENTER";
}

std::string expected_outer_string(
	const std::optional<DrivingDirection> &direction) {

	if (!direction.has_value()) {
		return "UNKNOWN";
	}

	if (*direction == DrivingDirection::CLOCKWISE) {

		return "LEFT";
	}

	return "RIGHT";
}

std::string turn_direction_to_string(
	const std::optional<DrivingDirection> &direction) {

	if (!direction.has_value()) {
		return "UNKNOWN";
	}

	return *direction == DrivingDirection::CLOCKWISE ? "RIGHT" : "LEFT";
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
	// LiDAR mounted 180 degrees.

	return {-point.distance_m * std::sin(rad),

		-point.distance_m * std::cos(rad)};
}

// =============================================================================
// GRID
// =============================================================================

void draw_grid(cv::Mat &map) {

	constexpr float GRID_M = 0.25f;

	const int spacing = static_cast<int>(GRID_M * SCALE_PX_PER_M);

	for (int x = static_cast<int>(ORIGIN.x); x < WORLD_VIEW_WIDTH;
		 x += spacing) {

		cv::line(map, {x, 0}, {x, MAP_HEIGHT}, cv::Scalar(25, 25, 25), 1);
	}

	for (int x = static_cast<int>(ORIGIN.x); x >= 0; x -= spacing) {

		cv::line(map, {x, 0}, {x, MAP_HEIGHT}, cv::Scalar(25, 25, 25), 1);
	}

	for (int y = static_cast<int>(ORIGIN.y); y < MAP_HEIGHT; y += spacing) {

		cv::line(map, {0, y}, {WORLD_VIEW_WIDTH, y}, cv::Scalar(25, 25, 25), 1);
	}

	for (int y = static_cast<int>(ORIGIN.y); y >= 0; y -= spacing) {

		cv::line(map, {0, y}, {WORLD_VIEW_WIDTH, y}, cv::Scalar(25, 25, 25), 1);
	}

	cv::line(map, {WORLD_VIEW_WIDTH, 0}, {WORLD_VIEW_WIDTH, MAP_HEIGHT},
		cv::Scalar(75, 75, 75), 2);
}

// =============================================================================
// RAW POINT
// =============================================================================

void draw_raw_points(cv::Mat &map, const TimedLidarData &scan) {

	for (const auto &point : scan.points) {

		if (point.distance_m <= 0.0f) {

			continue;
		}

		const cv::Point pixel = world_to_pixel(raw_to_cartesian(point));

		if (!is_inside_world_view(pixel)) {
			continue;
		}

		cv::circle(map, pixel, 1, cv::Scalar(80, 80, 80), -1);
	}
}

// =============================================================================
// SEGMENTS
// =============================================================================

void draw_segments(cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	for (const auto &segment : processed.line_segments) {

		cv::line(map, world_to_pixel(segment.start),
			world_to_pixel(segment.end), cv::Scalar(130, 130, 130), 1,
			cv::LINE_AA);
	}
}

// =============================================================================
// WALL
// =============================================================================

void draw_wall(cv::Mat &map, const std::optional<lidar::LineSegment> &wall,
	const cv::Scalar &color, const std::string &name) {

	if (!wall.has_value()) {
		return;
	}

	cv::line(map, world_to_pixel(wall->start), world_to_pixel(wall->end), color,
		4, cv::LINE_AA);

	const cv::Point2f center = (wall->start + wall->end) * 0.5f;

	cv::putText(map,
		name + " " + fixed(wall->perpendicular_distance(), 2) + "m",
		world_to_pixel(center) + cv::Point(8, -8), cv::FONT_HERSHEY_SIMPLEX,
		0.45, color, 1, cv::LINE_AA);
}

void draw_walls(cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	// LEFT
	draw_wall(map, processed.walls.left, cv::Scalar(255, 255, 0), "LEFT");

	// RIGHT
	draw_wall(map, processed.walls.right, cv::Scalar(0, 255, 0), "RIGHT");

	// FRONT
	draw_wall(map, processed.walls.front, cv::Scalar(0, 255, 255), "FRONT");
}

// =============================================================================
// OUTER WALL
//
// CW  -> LEFT outer
// CCW -> RIGHT outer
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
		world_to_pixel((*outer)->end), cv::Scalar(255, 0, 255), 8, cv::LINE_AA);

	const cv::Point2f center = ((*outer)->start + (*outer)->end) * 0.5f;

	cv::putText(map, "OUTER", world_to_pixel(center) + cv::Point(8, 18),
		cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 0, 255), 2,
		cv::LINE_AA);
}

// =============================================================================
// ROBOT
// =============================================================================

void draw_robot(cv::Mat &map) {

	const cv::Point origin(
		static_cast<int>(ORIGIN.x), static_cast<int>(ORIGIN.y));

	cv::circle(map, origin, 7, cv::Scalar(255, 255, 255), -1);

	// FRONT +Y
	cv::arrowedLine(map, origin, origin + cv::Point(0, -65),
		cv::Scalar(255, 255, 255), 2, cv::LINE_AA, 0, 0.2);

	cv::putText(map, "+Y FRONT", origin + cv::Point(10, -70),
		cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1,
		cv::LINE_AA);

	// RIGHT +X
	cv::arrowedLine(map, origin, origin + cv::Point(60, 0),
		cv::Scalar(255, 255, 255), 1, cv::LINE_AA, 0, 0.2);

	cv::putText(map, "+X RIGHT", origin + cv::Point(65, 5),
		cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1,
		cv::LINE_AA);
}

// =============================================================================
// STEERING DEBUG
//
// Visualization only.
// NO SERVO OUTPUT.
// =============================================================================

void draw_steering_arrow(cv::Mat &map, float steering_rad) {

	const cv::Point origin(
		static_cast<int>(ORIGIN.x), static_cast<int>(ORIGIN.y));

	constexpr float LENGTH = 100.0f;

	const int dx = static_cast<int>(std::sin(steering_rad) * LENGTH);

	const int dy = static_cast<int>(-std::cos(steering_rad) * LENGTH);

	const cv::Point end = origin + cv::Point(dx, dy);

	cv::arrowedLine(
		map, origin, end, cv::Scalar(0, 128, 255), 4, cv::LINE_AA, 0, 0.20);
}

void draw_heading_arrow(cv::Mat &map, float relative_heading_rad,
	const cv::Scalar &color, float length_px, const std::string &label) {

	const cv::Point origin(
		static_cast<int>(ORIGIN.x), static_cast<int>(ORIGIN.y));

	// OTOS heading is positive counter-clockwise. In the robot view +Y points
	// upward and +X points right, so positive relative heading points left.
	const int dx =
		static_cast<int>(-std::sin(relative_heading_rad) * length_px);
	const int dy =
		static_cast<int>(-std::cos(relative_heading_rad) * length_px);
	const cv::Point end = origin + cv::Point(dx, dy);

	cv::arrowedLine(map, origin, end, color, 2, cv::LINE_AA, 0, 0.18);
	cv::putText(map, label, end + cv::Point(6, -6), cv::FONT_HERSHEY_SIMPLEX,
		0.42, color, 1, cv::LINE_AA);
}

// =============================================================================
// FRONT DISTANCE
// =============================================================================

std::optional<float> get_front_distance(
	const lidar::ProcessedLidarData &processed) {

	if (!processed.walls.front) {

		return std::nullopt;
	}

	return processed.walls.front->perpendicular_distance();
}

// =============================================================================
// AUTOMATIC SIGN CHECK
//
// Only meaningful when:
// NORMAL mode
// outer wall valid
// wall angle near zero
//
// Because Stanley = heading error + cross track term.
// =============================================================================

std::string sign_check(
	const navigation::NavigationResult &nav, navigation::NavigationMode mode) {

	if (mode != navigation::NavigationMode::NORMAL) {

		return "WAIT NORMAL";
	}

	if (!nav.debug.outer_wall_valid) {

		return "NO OUTER";
	}

	constexpr float MAX_HEADING_ERROR = 5.0f * PI / 180.0f;

	if (std::abs(nav.debug.angle_error_rad) > MAX_HEADING_ERROR) {

		return "ALIGN ROBOT";
	}

	constexpr float CTE_EPS = 0.005f;

	constexpr float STEER_EPS = 0.01f;

	const float cte = nav.debug.distance_error_m;

	const float steering = nav.command.steering_rad;

	if (std::abs(cte) < CTE_EPS) {

		return "CTE CENTER";
	}

	if (std::abs(steering) < STEER_EPS) {

		return "STEER CENTER";
	}

	const bool same_sign =
		(cte > 0.0f && steering > 0.0f) || (cte < 0.0f && steering < 0.0f);

	return same_sign ? "PASS" : "CHECK";
}

// =============================================================================
// DEBUG PANEL
// =============================================================================

void draw_debug_panel(cv::Mat &map, const lidar::ProcessedLidarData &processed,
	const navigation::NavigationResult &nav,
	const navigation::NavigationState &state, float heading_rad,
	float speed_mps, float wall_correction_rad, float target_outer_distance_m,
	float turn_trigger_distance_m, float max_steering_rad, int total_turns,
	std::uint64_t frame_diff_us, std::int64_t process_us) {

	const cv::Scalar white(235, 235, 235);
	const cv::Scalar dim(150, 150, 150);
	const cv::Scalar cyan(255, 255, 0);
	const cv::Scalar orange(0, 155, 255);
	const cv::Scalar magenta(255, 0, 255);
	const cv::Scalar green(70, 255, 70);
	const cv::Scalar yellow(0, 255, 255);

	cv::rectangle(map, {WORLD_VIEW_WIDTH + 2, 0}, {MAP_WIDTH, MAP_HEIGHT},
		cv::Scalar(12, 12, 16), -1);

	int x = WORLD_VIEW_WIDTH + 20;
	int y = 30;
	constexpr int DY = 24;

	auto text = [&](const std::string &s,
					const cv::Scalar &color = cv::Scalar(235, 235, 235),
					float scale = 0.49f, int thickness = 1) {
		cv::putText(map, s, {x, y}, cv::FONT_HERSHEY_SIMPLEX, scale, color,
			thickness, cv::LINE_AA);
		y += DY;
	};

	auto section = [&](const std::string &title) {
		y += 7;
		text(title, cv::Scalar(110, 190, 255), 0.43f, 1);
		cv::line(map, {x, y - 17}, {MAP_WIDTH - 18, y - 17},
			cv::Scalar(55, 55, 65), 1);
	};

	const auto front_distance = get_front_distance(processed);
	const std::string front =
		front_distance ? fixed(*front_distance, 3) + " m" : "NONE";
	const float effective_trigger_m = nav.debug.effective_turn_trigger_m > 0.0f
		? nav.debug.effective_turn_trigger_m
		: turn_trigger_distance_m;
	const std::string check = sign_check(nav, state.mode);
	const cv::Scalar mode_color =
		state.mode == navigation::NavigationMode::NORMAL	? green
		: state.mode == navigation::NavigationMode::TURNING ? orange
															: yellow;
	const cv::Scalar check_color = check == "PASS" ? green
		: check == "CHECK"						   ? cv::Scalar(60, 60, 255)
												   : dim;

	text("NAVIGATION CONTROLLER", white, 0.62f, 2);
	text("MONITOR ONLY - ACTUATORS DISCONNECTED", cv::Scalar(80, 80, 255),
		0.43f, 1);

	section("STATE");
	text("MODE       " + mode_to_string(state.mode), mode_color, 0.57f, 2);
	text("DIRECTION  " + direction_to_string(state.direction), white);
	text("NEXT TURN  " + turn_direction_to_string(state.direction) +
			"   ARMED " + (state.turn_armed ? "YES" : "NO"),
		state.turn_armed ? green : dim);
	text("TURN " + std::to_string(state.turn_count) + "/" +
			std::to_string(total_turns) + "   LAP " +
			std::to_string(state.lap) + "   CORNER " +
			std::to_string(state.corner_index),
		white);
	text("HEADING CONFIRM FRAMES  " +
			std::to_string(state.heading_confirm_frames),
		dim);

	section("STEERING  (positive = RIGHT)");
	text("OUTPUT  " + fixed(nav.command.steering_rad * RAD_TO_DEG, 1) +
			" deg  " + steering_to_string(nav.command.steering_rad),
		orange, 0.58f, 2);
	text("RAW     " + fixed(nav.debug.raw_steering_rad * RAD_TO_DEG, 1) +
			" deg   FF " +
			fixed(nav.debug.turn_feedforward_rad * RAD_TO_DEG, 1) + " deg",
		white);
	text("LIMIT   +/-" + fixed(max_steering_rad * RAD_TO_DEG, 1) +
			" deg   SERVO OFF",
		dim);

	section("HEADING  (OTOS positive = CCW)");
	text("CURRENT " + fixed(heading_rad * RAD_TO_DEG, 1) + " deg   TARGET " +
			fixed(state.target_heading_rad * RAD_TO_DEG, 1) + " deg",
		white);
	if (state.mode == navigation::NavigationMode::TURNING) {
		text("MOVING REF " +
				fixed(nav.debug.turn_reference_heading_rad * RAD_TO_DEG, 1) +
				" deg   TRACK ERR " +
				fixed(nav.debug.heading_tracking_error_rad * RAD_TO_DEG, 1) +
				" deg",
			cyan);
	} else {
		text("MOVING REF --   TRACK ERR --", dim);
	}
	text("TURN ERR " + fixed(nav.debug.heading_error_rad * RAD_TO_DEG, 1) +
			" deg   PROGRESS " + fixed(nav.debug.turn_progress * 100.0f, 0) +
			"%",
		white);
	text("LIDAR HEADING CORR  " + fixed(wall_correction_rad * RAD_TO_DEG, 1) +
			" deg",
		dim);

	section("SPEED PROFILE");
	text("OTOS " + fixed(speed_mps, 2) + " m/s   OUTPUT " +
			fixed(nav.command.target_speed_mps, 2) + " m/s",
		white);
	text("RAW TARGET " + fixed(nav.debug.raw_target_speed_mps, 2) +
			" m/s   CORNER CAP " + fixed(nav.debug.corner_speed_mps, 2) +
			" m/s",
		white);
	text("MOTOR OUTPUT OFF", dim);

	section("LIDAR / WALL FOLLOWING");
	text("OUTER " + expected_outer_string(state.direction) + "   VALID " +
			(nav.debug.outer_wall_valid ? "YES" : "NO"),
		nav.debug.outer_wall_valid ? magenta : cv::Scalar(60, 60, 255));
	text("DIST " + fixed(nav.debug.outer_distance_m, 3) + " m   TARGET " +
			fixed(target_outer_distance_m, 3) + " m",
		white);
	text("CTE " + fixed(nav.debug.distance_error_m, 3) + " m  " +
			cte_to_string(nav.debug.distance_error_m) + "   WALL ERR " +
			fixed(nav.debug.angle_error_rad * RAD_TO_DEG, 1) + " deg",
		white);
	text("SIGN CHECK  " + check, check_color);
	text("FRONT " + front + "   VALID " +
			(nav.debug.front_wall_valid ? "YES" : "NO"),
		nav.debug.front_wall_valid ? yellow : dim);
	text("DYNAMIC TURN TRIGGER  " + fixed(effective_trigger_m, 3) + " m",
		yellow);

	section("TIMING");
	text("FRAME " + fixed(static_cast<float>(frame_diff_us) / 1000.0f, 1) +
			" ms   PROCESS " +
			fixed(static_cast<float>(process_us) / 1000.0f, 2) + " ms",
		dim);
	text("CONTROLLER DT  " + fixed(nav.debug.update_dt_s * 1000.0f, 1) + " ms",
		dim);
}

} // namespace

// =============================================================================
// MAIN
// =============================================================================

int main() {

	// =========================================================================
	// HARDWARE
	// =========================================================================

	lidar::LidarModule lidar("/dev/ttyAMA0", 1000000);

	lidar::LidarProcessor lidar_processor;

	otos::OTOS otos;

	// =========================================================================
	// NAVIGATION CONFIG
	// =========================================================================

	navigation::NavigationConfig nav_config;

	// Follow outer wall at 30 cm.
	nav_config.target_outer_distance_m = 0.30f;

	// Stanley
	nav_config.stanley.k = 0.85f;

	nav_config.stanley.softening_speed_mps = 0.30f;

	nav_config.stanley.max_steering_rad = 30.0f * PI / 180.0f;

	// Search direction centering.
	nav_config.search_center_kp = 0.8f;

	// Corner detection.
	nav_config.approach_distance_m = 0.90f;

	nav_config.turn_trigger_distance_m = 0.50f;

	nav_config.turn_rearm_distance_m = 0.85f;

	nav_config.turn_preview_time_s = 0.10f;

	nav_config.turn_trigger_confirm_frames = 2;

	// Measure this on the final chassis before track tuning.
	nav_config.wheelbase_m = 0.18f;

	// 0.10 m outer-wall radius + 0.30 m wall-following offset.
	nav_config.corner_radius_m = 0.40f;

	nav_config.turn_entry_blend_rad = 10.0f * PI / 180.0f;

	nav_config.turn_exit_blend_rad = 22.0f * PI / 180.0f;

	nav_config.exit_acceleration_blend_rad = 15.0f * PI / 180.0f;

	// Heading turn controller.
	nav_config.turn_heading_kp = 0.8f;

	nav_config.heading_tolerance_rad = 3.0f * PI / 180.0f;

	nav_config.heading_confirm_frames = 3;

	// OTOS:
	// +heading = CCW
	nav_config.clockwise_turn_delta_rad = -PI * 0.5f;

	nav_config.counter_clockwise_turn_delta_rad = PI * 0.5f;

	// OTOS heading convention
	// ->
	// steering convention
	//
	// steering -
	// = LEFT
	//
	// steering +
	// = RIGHT
	nav_config.heading_to_steering_sign = -1.0f;

	// These are only Navigation outputs.
	// No motor is connected in this test.
	nav_config.search_speed_mps = 0.25f;

	nav_config.normal_speed_mps = 0.85f;

	nav_config.approach_speed_mps = 0.72f;

	nav_config.turning_speed_mps = 0.65f;

	nav_config.lost_wall_speed_mps = 0.30f;

	nav_config.max_lateral_acceleration_mps2 = 1.40f;

	nav_config.steering_filter_time_constant_s = 0.035f;

	nav_config.max_steering_rate_rad_s = 7.0f;

	nav_config.max_acceleration_mps2 = 1.8f;

	nav_config.max_deceleration_mps2 = 3.0f;

	nav_config.total_turns = 12;

	// =========================================================================
	// DIRECTION CONFIG
	// =========================================================================

	navigation::InitialDirectionConfig direction_config;

	direction_config.max_collinear_offset_m = 0.04f;

	direction_config.max_continuation_gap_m = 0.20f;

	direction_config.max_perpendicular_error_rad = 15.0f * PI / 180.0f;

	direction_config.max_connection_gap_m = 0.35f;

	direction_config.min_candidate_length_m = 0.15f;

	direction_config.frame_min_score = 6.0f;

	direction_config.frame_score_margin = 1.5f;

	direction_config.score_decay = 0.7f;

	direction_config.required_confirm_frames = 3;

	navigation::NavigationController navigation(nav_config, direction_config);

	// =========================================================================
	// LIDAR
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

	// =========================================================================
	// OTOS
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

	std::cout << "LiDAR + OTOS ready\n";

	// =========================================================================
	// DEBUG WINDOW
	// =========================================================================

	cv::Mat debug_map(MAP_HEIGHT, MAP_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

	bool nav_initialized = false;

	std::uint64_t previous_timestamp_us = 0;

	navigation::NavigationMode previous_mode =
		navigation::NavigationMode::SEARCH_DIRECTION;

	std::optional<DrivingDirection> previous_direction;

	std::cout << "\n"
			  << "==========================================\n"
			  << " NavigationController TEST\n"
			  << "==========================================\n"
			  << "NO MOTOR OUTPUT\n"
			  << "NO SERVO OUTPUT\n"
			  << "\n"
			  << "R = reset navigation\n"
			  << "Q / ESC = quit\n"
			  << "==========================================\n\n";

	// =========================================================================
	// LOOP
	// =========================================================================

	while (true) {

		TimedLidarData scan;

		sfe_otos_pose2d_t pos{};
		sfe_otos_pose2d_t vel{};
		sfe_otos_pose2d_t acc{};

		// ---------------------------------------------------------------------
		// LIDAR
		// ---------------------------------------------------------------------

		if (!lidar.wait_for_data(scan)) {

			break;
		}

		if (scan.points.empty()) {
			continue;
		}

		// ---------------------------------------------------------------------
		// OTOS
		// ---------------------------------------------------------------------

		const sfTkError_t otos_error = otos.getPosVelAcc(pos, vel, acc);

		if (otos_error != ksfTkErrOk) {

			std::cerr << "OTOS read error: " << static_cast<int>(otos_error)
					  << '\n';

			continue;
		}

		const float heading_rad = pos.h;

		// Magnitude is enough for Stanley test.
		const float speed_mps = std::hypot(vel.x, vel.y);

		// ---------------------------------------------------------------------
		// INIT NAV
		// ---------------------------------------------------------------------

		if (!nav_initialized) {

			navigation.reset(heading_rad);

			previous_mode = navigation.state().mode;

			nav_initialized = true;

			std::cout << "[NAV] initialized heading="
					  << heading_rad * RAD_TO_DEG << " deg\n";
		}

		// ---------------------------------------------------------------------
		// HEADING CORRECTION FOR LIDAR WALL RESOLVER
		// ---------------------------------------------------------------------

		const float wall_correction_rad = normalize_angle(
			heading_rad - navigation.state().target_heading_rad);

		// ---------------------------------------------------------------------
		// PROCESS
		// ---------------------------------------------------------------------

		const auto process_start = std::chrono::steady_clock::now();

		const auto processed =
			lidar_processor.process(scan, wall_correction_rad,
				4,		// min_segment_point
				0.035f, // max_line_error_m
				0.12f,	// max_point_gap_m
				5.0f,	// max_angle_diff deg
				0.04f,	// max_collinear_error_m
				0.10f	// max_segment_gap_m
			);

		// ---------------------------------------------------------------------
		// REAL NavigationController
		// ---------------------------------------------------------------------

		const auto nav_result =
			navigation.update(processed, heading_rad, speed_mps);

		const auto process_end = std::chrono::steady_clock::now();

		const auto process_us =
			std::chrono::duration_cast<std::chrono::microseconds>(
				process_end - process_start)
				.count();

		const auto &state = navigation.state();

		// ---------------------------------------------------------------------
		// MODE TRANSITION LOG
		// ---------------------------------------------------------------------

		if (state.mode != previous_mode) {

			std::cout << "\n[MODE] " << mode_to_string(previous_mode) << " -> "
					  << mode_to_string(state.mode) << '\n';

			previous_mode = state.mode;
		}

		// ---------------------------------------------------------------------
		// DIRECTION LOCK LOG
		// ---------------------------------------------------------------------

		if (state.direction != previous_direction) {

			std::cout << "[DIRECTION] " << direction_to_string(state.direction)
					  << '\n';

			previous_direction = state.direction;
		}

		// ---------------------------------------------------------------------
		// FRAME TIME
		// ---------------------------------------------------------------------

		std::uint64_t frame_diff_us = 0;

		if (previous_timestamp_us != 0) {

			frame_diff_us = scan.timestamp_us - previous_timestamp_us;
		}

		previous_timestamp_us = scan.timestamp_us;

		// ---------------------------------------------------------------------
		// DRAW
		// ---------------------------------------------------------------------

		debug_map.setTo(cv::Scalar(0, 0, 0));

		draw_grid(debug_map);

		draw_raw_points(debug_map, scan);

		draw_segments(debug_map, processed);

		draw_walls(debug_map, processed);

		// The selected outer wall is drawn THICK MAGENTA.
		draw_outer_wall(debug_map, processed, state.direction);

		draw_robot(debug_map);

		// Visualization only.
		// Absolutely no servo command is sent.
		draw_steering_arrow(debug_map, nav_result.command.steering_rad);
		draw_heading_arrow(debug_map,
			normalize_angle(state.target_heading_rad - heading_rad),
			cv::Scalar(255, 0, 255), 145.0f, "TARGET H");

		if (state.mode == navigation::NavigationMode::TURNING) {
			draw_heading_arrow(debug_map,
				normalize_angle(
					nav_result.debug.turn_reference_heading_rad - heading_rad),
				cv::Scalar(255, 255, 0), 120.0f, "MOVING REF");
		}

		draw_debug_panel(debug_map, processed, nav_result, state, heading_rad,
			speed_mps, wall_correction_rad, nav_config.target_outer_distance_m,
			nav_config.turn_trigger_distance_m,
			nav_config.stanley.max_steering_rad, nav_config.total_turns,
			frame_diff_us, process_us);

		cv::imshow("NavigationController Dashboard - NO ACTUATORS", debug_map);

		// ---------------------------------------------------------------------
		// KEY
		// ---------------------------------------------------------------------

		const int key = cv::waitKey(1);

		if (key == 'q' || key == 'Q' || key == 27) {

			break;
		}

		if (key == 'r' || key == 'R') {

			navigation.reset(heading_rad);

			previous_mode = navigation.state().mode;

			previous_direction.reset();

			std::cout << "\n[NAV] RESET\n";
		}
	}

	// =========================================================================
	// STOP
	// =========================================================================

	std::cout << "\nStopping...\n";

	lidar.stop();

	cv::destroyAllWindows();

	return 0;
}
