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

constexpr int MAP_WIDTH = 1000;
constexpr int MAP_HEIGHT = 900;

constexpr float SCALE_PX_PER_M = 300.0f;

constexpr float PI = 3.14159265358979323846f;
constexpr float RAD_TO_DEG = 180.0f / PI;

const cv::Point2f ORIGIN(static_cast<float>(MAP_WIDTH) * 0.50f,
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

bool is_inside_map(const cv::Point &p) {

	return p.x >= 0 && p.x < MAP_WIDTH && p.y >= 0 && p.y < MAP_HEIGHT;
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

	for (int x = static_cast<int>(ORIGIN.x); x < MAP_WIDTH; x += spacing) {

		cv::line(map, {x, 0}, {x, MAP_HEIGHT}, cv::Scalar(25, 25, 25), 1);
	}

	for (int x = static_cast<int>(ORIGIN.x); x >= 0; x -= spacing) {

		cv::line(map, {x, 0}, {x, MAP_HEIGHT}, cv::Scalar(25, 25, 25), 1);
	}

	for (int y = static_cast<int>(ORIGIN.y); y < MAP_HEIGHT; y += spacing) {

		cv::line(map, {0, y}, {MAP_WIDTH, y}, cv::Scalar(25, 25, 25), 1);
	}

	for (int y = static_cast<int>(ORIGIN.y); y >= 0; y -= spacing) {

		cv::line(map, {0, y}, {MAP_WIDTH, y}, cv::Scalar(25, 25, 25), 1);
	}
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

		if (!is_inside_map(pixel)) {
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
	float turn_trigger_distance_m, std::uint64_t frame_diff_us,
	std::int64_t process_us) {

	const std::string mode = mode_to_string(state.mode);

	const std::string direction = direction_to_string(state.direction);

	const std::string outer = expected_outer_string(state.direction);

	const float cte = nav.debug.distance_error_m;

	const float steering = nav.command.steering_rad;

	const auto front_distance = get_front_distance(processed);

	const std::string front =
		front_distance ? fixed(*front_distance, 3) + "m" : "NONE";

	const std::string check = sign_check(nav, state.mode);

	const std::string cte_expected = cte_to_string(cte);

	const std::string steering_direction = steering_to_string(steering);

	int x = 20;

	int y = 28;

	constexpr int DY = 25;

	auto text = [&](const std::string &s, const cv::Scalar &color) {
		cv::putText(map, s, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.52,
			color, 1, cv::LINE_AA);

		y += DY;
	};

	// ---------------------------------------------------------
	// Navigation mode
	// ---------------------------------------------------------

	text("MODE: " + mode,
		state.mode == navigation::NavigationMode::NORMAL
			? cv::Scalar(0, 255, 0)
			: cv::Scalar(0, 255, 255));

	// ---------------------------------------------------------
	// Direction
	// ---------------------------------------------------------

	text("DIRECTION: " + direction, cv::Scalar(255, 255, 255));

	// ---------------------------------------------------------
	// Outer wall mapping
	// ---------------------------------------------------------

	text("OUTER EXPECT: " + outer +
			"   VALID: " + (nav.debug.outer_wall_valid ? "YES" : "NO"),
		cv::Scalar(255, 0, 255));

	// ---------------------------------------------------------
	// Outer distance
	// ---------------------------------------------------------

	text("OUTER DIST: " + fixed(nav.debug.outer_distance_m, 3) +
			"m   TARGET: " + fixed(target_outer_distance_m, 3) + "m",
		cv::Scalar(255, 255, 255));

	// ---------------------------------------------------------
	// Cross track
	// ---------------------------------------------------------

	text("CTE: " + fixed(cte, 3) + "m   CTE EXPECT: " + cte_expected,
		cte > 0.0f ? cv::Scalar(0, 200, 255) : cv::Scalar(255, 200, 0));

	// ---------------------------------------------------------
	// Wall heading
	// ---------------------------------------------------------

	text("WALL ERR: " + fixed(nav.debug.angle_error_rad * RAD_TO_DEG, 2) +
			" deg",
		cv::Scalar(255, 255, 255));

	// ---------------------------------------------------------
	// Stanley steering
	// ---------------------------------------------------------

	text("STEERING: " + fixed(steering, 3) + " rad / " +
			fixed(steering * RAD_TO_DEG, 1) + " deg   => " + steering_direction,
		cv::Scalar(0, 128, 255));

	text("RAW STEER: " + fixed(nav.debug.raw_steering_rad * RAD_TO_DEG, 1) +
			" deg   FF: " +
			fixed(nav.debug.turn_feedforward_rad * RAD_TO_DEG, 1) + " deg",
		cv::Scalar(0, 160, 220));

	// ---------------------------------------------------------
	// CTE sign check
	// ---------------------------------------------------------

	const cv::Scalar check_color = check == "PASS" ? cv::Scalar(0, 255, 0)
		: check == "CHECK"						   ? cv::Scalar(0, 0, 255)
												   : cv::Scalar(150, 150, 150);

	text("CTE -> STEERING SIGN CHECK: " + check, check_color);

	// ---------------------------------------------------------
	// Front
	// ---------------------------------------------------------

	const float effective_trigger_m = nav.debug.effective_turn_trigger_m > 0.0f
		? nav.debug.effective_turn_trigger_m
		: turn_trigger_distance_m;

	text("FRONT: " + front +
			"   TURN TRIGGER: " + fixed(effective_trigger_m, 2) + "m",
		cv::Scalar(0, 255, 255));

	// ---------------------------------------------------------
	// OTOS heading
	// ---------------------------------------------------------

	text("OTOS H: " + fixed(heading_rad * RAD_TO_DEG, 1) + " deg   TARGET H: " +
			fixed(state.target_heading_rad * RAD_TO_DEG, 1) + " deg",
		cv::Scalar(255, 255, 255));

	// ---------------------------------------------------------
	// Turning heading error
	// ---------------------------------------------------------

	text("TURN ERR: " + fixed(nav.debug.heading_error_rad * RAD_TO_DEG, 1) +
			" deg   TRACK: " +
			fixed(nav.debug.heading_tracking_error_rad * RAD_TO_DEG, 1) +
			" deg",
		cv::Scalar(255, 255, 255));

	text("TURN PROGRESS: " + fixed(nav.debug.turn_progress * 100.0f, 0) +
			"%   CORNER SPEED: " + fixed(nav.debug.corner_speed_mps, 2) +
			" m/s",
		cv::Scalar(255, 255, 255));

	// ---------------------------------------------------------
	// Speed
	// ---------------------------------------------------------

	text("OTOS SPEED: " + fixed(speed_mps, 2) + " m/s   NAV TARGET: " +
			fixed(nav.command.target_speed_mps, 2) + " m/s [NOT ACTUATED]",
		cv::Scalar(120, 120, 120));

	// ---------------------------------------------------------
	// Wall correction
	// ---------------------------------------------------------

	text("LIDAR HEADING CORR: " + fixed(wall_correction_rad * RAD_TO_DEG, 1) +
			" deg",
		cv::Scalar(120, 120, 120));

	// ---------------------------------------------------------
	// Progress
	// ---------------------------------------------------------

	text("TURN: " + std::to_string(state.turn_count) +
			"   LAP: " + std::to_string(state.lap) +
			"   CORNER: " + std::to_string(state.corner_index),
		cv::Scalar(255, 255, 255));

	text("FRAME: " + fixed(static_cast<float>(frame_diff_us) / 1000.0f, 1) +
			"ms   PROCESS: " +
			fixed(static_cast<float>(process_us) / 1000.0f, 2) +
			"ms   NAV DT: " + fixed(nav.debug.update_dt_s * 1000.0f, 1) + "ms",
		cv::Scalar(120, 120, 120));
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

		draw_debug_panel(debug_map, processed, nav_result, state, heading_rad,
			speed_mps, wall_correction_rad, nav_config.target_outer_distance_m,
			nav_config.turn_trigger_distance_m, frame_diff_us, process_us);

		cv::imshow("NavigationController TEST - NO ACTUATORS", debug_map);

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
