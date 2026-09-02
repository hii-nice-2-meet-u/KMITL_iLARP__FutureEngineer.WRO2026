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
#include "telemetry_logger.hpp"
#include "track_map.hpp"
#include "wall_logger.hpp"

namespace {

constexpr int WORLD_VIEW_WIDTH = 920;
constexpr int PANEL_WIDTH = 500;
constexpr int MAP_WIDTH = WORLD_VIEW_WIDTH + PANEL_WIDTH;
constexpr int MAP_HEIGHT = 980;

constexpr float SCALE_PX_PER_M = 300.0f;

constexpr float PI = 3.14159265358979323846f;
constexpr float RAD_TO_DEG = 180.0f / PI;

const cv::Point2f ORIGIN(static_cast<float>(WORLD_VIEW_WIDTH) * 0.50f,
	static_cast<float>(MAP_HEIGHT) * 0.68f);

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

cv::Point2f raw_to_cartesian(const LidarPoint &point) {

	const float rad = point.angle_deg * PI / 180.0f;

	return {-point.distance_m * std::sin(rad),

		-point.distance_m * std::cos(rad)};
}

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

void draw_segments(cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	for (const auto &segment : processed.line_segments) {

		cv::line(map, world_to_pixel(segment.start),
			world_to_pixel(segment.end), cv::Scalar(130, 130, 130), 1,
			cv::LINE_AA);
	}
}

void draw_wall(cv::Mat &map, const std::optional<lidar::LineSegment> &wall,
	const cv::Scalar &color, const std::string &name) {

	if (!wall.has_value()) {
		return;
	}

	cv::line(map, world_to_pixel(wall->start), world_to_pixel(wall->end), color,
		4, cv::LINE_AA);

	const cv::Point2f center = (wall->start + wall->end) * 0.5f;

	cv::putText(map,
		name + " DIST " + fixed(wall->perpendicular_distance(), 2) +
			"m LEN " + fixed(wall->length(), 2) + "m",
		world_to_pixel(center) + cv::Point(8, -8), cv::FONT_HERSHEY_SIMPLEX,
		0.45, color, 1, cv::LINE_AA);
}

void draw_walls(cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	draw_wall(map, processed.walls.left, cv::Scalar(255, 255, 0), "LEFT");

	draw_wall(map, processed.walls.right, cv::Scalar(0, 255, 0), "RIGHT");

	draw_wall(map, processed.walls.front, cv::Scalar(0, 255, 255), "FRONT");
}

void draw_obstacles(cv::Mat &map, const lidar::ProcessedLidarData &processed) {

	const cv::Scalar color(0, 128, 255);

	for (std::size_t i = 0; i < processed.obstacles.size(); ++i) {

		const auto &obstacle = processed.obstacles[i];
		const cv::Point start = world_to_pixel(obstacle.start());
		const cv::Point end = world_to_pixel(obstacle.end());
		const cv::Point center = world_to_pixel(obstacle.center);

		if (!is_inside_world_view(center)) {
			continue;
		}

		cv::line(map, start, end, color, 6, cv::LINE_AA);
		cv::circle(map, start, 4, color, -1, cv::LINE_AA);
		cv::circle(map, end, 4, color, -1, cv::LINE_AA);
		cv::circle(map, center, 7, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

		const std::string label = "OBS" + std::to_string(i) + " " +
			fixed(obstacle.distance_m(), 2) + "m " +
			fixed(obstacle.bearing_rad() * RAD_TO_DEG, 1) + "deg " +
			fixed(obstacle.width_m * 100.0f, 1) + "cm";

		cv::putText(map, label, center + cv::Point(10, -10),
			cv::FONT_HERSHEY_SIMPLEX, 0.43, color, 1, cv::LINE_AA);
	}
}

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

void draw_robot(cv::Mat &map) {

	const cv::Point origin(
		static_cast<int>(ORIGIN.x), static_cast<int>(ORIGIN.y));

	cv::circle(map, origin, 7, cv::Scalar(255, 255, 255), -1);

	cv::arrowedLine(map, origin, origin + cv::Point(0, -65),
		cv::Scalar(255, 255, 255), 2, cv::LINE_AA, 0, 0.2);

	cv::putText(map, "+Y FRONT", origin + cv::Point(10, -70),
		cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1,
		cv::LINE_AA);

	cv::arrowedLine(map, origin, origin + cv::Point(60, 0),
		cv::Scalar(255, 255, 255), 1, cv::LINE_AA, 0, 0.2);

	cv::putText(map, "+X RIGHT", origin + cv::Point(65, 5),
		cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1,
		cv::LINE_AA);
}

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

	const int dx =
		static_cast<int>(-std::sin(relative_heading_rad) * length_px);
	const int dy =
		static_cast<int>(-std::cos(relative_heading_rad) * length_px);
	const cv::Point end = origin + cv::Point(dx, dy);

	cv::arrowedLine(map, origin, end, color, 2, cv::LINE_AA, 0, 0.18);
	cv::putText(map, label, end + cv::Point(6, -6), cv::FONT_HERSHEY_SIMPLEX,
		0.42, color, 1, cv::LINE_AA);
}

void draw_wall_corner(cv::Mat &map, const navigation::NavigationDebug &debug,
	float lidar_lateral_offset_m, float lidar_forward_offset_m) {
	if (!debug.wall_corner_candidate_valid && !debug.wall_corner_confirmed) {
		return;
	}

	const cv::Point2f lidar_point{
		debug.wall_corner_lateral_m - lidar_lateral_offset_m,
		debug.wall_corner_forward_m - lidar_forward_offset_m};
	const cv::Point pixel = world_to_pixel(lidar_point);
	if (!is_inside_world_view(pixel)) {
		return;
	}

	const cv::Scalar color = debug.wall_corner_confirmed
		? cv::Scalar(70, 255, 70)
		: cv::Scalar(0, 255, 255);
	cv::circle(map, pixel, 10, color, 2, cv::LINE_AA);
	cv::line(map, pixel + cv::Point(-14, 0), pixel + cv::Point(14, 0), color, 2,
		cv::LINE_AA);
	cv::line(map, pixel + cv::Point(0, -14), pixel + cv::Point(0, 14), color, 2,
		cv::LINE_AA);
	cv::putText(map,
		debug.wall_corner_confirmed ? "WALL CORNER CONF" : "WALL CORNER CAND",
		pixel + cv::Point(14, -12), cv::FONT_HERSHEY_SIMPLEX, 0.43f, color, 1,
		cv::LINE_AA);
}

std::optional<float> get_front_distance(
	const lidar::ProcessedLidarData &processed) {

	if (!processed.walls.front) {

		return std::nullopt;
	}

	return processed.walls.front->perpendicular_distance();
}

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

void draw_debug_panel(cv::Mat &map, const lidar::ProcessedLidarData &processed,
	const navigation::NavigationResult &nav,
	const navigation::NavigationState &state, float heading_rad,
	float speed_mps, float target_outer_distance_m,
	float turn_trigger_distance_m, float max_steering_rad, int total_turns,
	const navigation::TrackMap &track_map,
	const std::optional<navigation::ReplayHint> &replay_hint,
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
	text("MAP  " +
			(track_map.ready_for_replay() ? std::string("REPLAY READY")
										  : std::string("LEARN ") +
						std::to_string(track_map.learned_corner_count()) +
						"/4"),
		track_map.ready_for_replay() ? green : yellow);
	if (replay_hint.has_value()) {
		text("NEXT MAP CORNER " + std::to_string(replay_hint->corner_index) +
				"   DIST " + fixed(replay_hint->distance_to_entry_m, 2) + " m" +
				(replay_hint->approach_recommended ? "   PREVIEW" : ""),
			replay_hint->approach_recommended ? orange : dim);
	} else {
		text("NEXT MAP CORNER  --", dim);
	}

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

	section("SPEED PROFILE");
	text("OTOS " + fixed(speed_mps, 2) + " m/s   OUTPUT " +
			fixed(nav.command.target_speed_mps, 2) + " m/s",
		white);
	text("RAW TARGET " + fixed(nav.debug.raw_target_speed_mps, 2) +
			" m/s   CORNER CAP " + fixed(nav.debug.corner_speed_mps, 2) +
			" m/s",
		white);
	text("LAP FACTOR x" + fixed(nav.debug.replay_speed_factor, 2) +
			"   NORMAL " + fixed(nav.debug.active_normal_speed_mps, 2) +
			"   APPROACH " + fixed(nav.debug.active_approach_speed_mps, 2),
		cyan);
	text("MOTOR OUTPUT OFF", dim);

	section("LIDAR / WALL FOLLOWING");
	text("OUTER " + expected_outer_string(state.direction) + "   VALID " +
			(nav.debug.outer_wall_valid ? "YES" : "NO"),
		nav.debug.outer_wall_valid ? magenta : cv::Scalar(60, 60, 255));
	text("FOLLOW " +
			std::string(nav.debug.corridor_center_active ? "CENTER" :
												 "OUTER FALLBACK"),
		nav.debug.corridor_center_active ? green : dim);
	text("OUTER " + fixed(nav.debug.outer_distance_m, 3) + " m   INNER " +
			fixed(nav.debug.inner_distance_m, 3) + " m",
		white);
	text("OUTER FALLBACK TARGET " + fixed(target_outer_distance_m, 3) + " m",
		dim);
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
	text("WALL CORNER  CAND " +
			(nav.debug.wall_corner_candidate_valid ? std::string("YES")
												   : "NO") +
			"   CONF " + (nav.debug.wall_corner_confirmed ? "YES" : "NO") +
			"   FRAMES " + std::to_string(nav.debug.wall_corner_confirm_frames),
		nav.debug.wall_corner_confirmed ? green : dim);
	text("CORNER FWD " + fixed(nav.debug.wall_corner_forward_m, 3) +
			" m   LAT " + fixed(nav.debug.wall_corner_lateral_m, 3) +
			" m   ERR " + fixed(nav.debug.wall_corner_stability_error_m, 3) +
			" m",
		white);
	text("TURN SOURCE  " +
			(nav.debug.wall_corner_confirmed ? std::string("INNER WALL CORNER")
					: nav.debug.front_wall_fallback_active
					? std::string("FRONT WALL FALLBACK")
					: std::string("LEGACY FRONT WALL")),
		nav.debug.wall_corner_confirmed ? green : yellow);
	text("OBSTACLES  " + std::to_string(processed.obstacles.size()),
		processed.obstacles.empty() ? dim : orange);

	section("TIMING");
	text("FRAME " + fixed(static_cast<float>(frame_diff_us) / 1000.0f, 1) +
			" ms   PROCESS " +
			fixed(static_cast<float>(process_us) / 1000.0f, 2) + " ms",
		dim);
	text("CONTROLLER DT  " + fixed(nav.debug.update_dt_s * 1000.0f, 1) + " ms",
		dim);
}

}

int main() {
	logging::install_stop_signal_handlers();

	lidar::LidarModule lidar("/dev/ttyAMA0", 1000000);

	lidar::LidarProcessor lidar_processor;

	otos::OTOS otos;

	navigation::NavigationConfig nav_config;
	nav_config.enable_replay_speed_factors = true;

	nav_config.target_outer_distance_m = 0.20f;

	nav_config.stanley.k = 0.85f;

	nav_config.stanley.heading_pid.kp = 1.00f;
	nav_config.stanley.heading_pid.ki = 0.12f;
	nav_config.stanley.heading_pid.kd = 0.025f;

	nav_config.turn_heading_pid.kp = 0.80f;
	nav_config.turn_heading_pid.ki = 0.08f;
	nav_config.turn_heading_pid.kd = 0.020f;


	nav_config.turn_trigger_distance_m = 0.85f;
	nav_config.use_wall_corner_trigger = true;
	nav_config.front_wall_fallback_distance_m = 0.40f;
	nav_config.lidar_lateral_offset_m = 0.0f;
	nav_config.lidar_forward_offset_m = 0.0f;
	nav_config.wall_corner_to_path_offset_m = 0.0f;

	nav_config.turn_entry_blend_rad = 13.0f * PI / 180.0f;
	nav_config.turn_exit_blend_rad = 26.0f * PI / 180.0f;

	nav_config.max_steering_rate_rad_s = 7.0f;

	nav_config.max_lateral_acceleration_mps2 = 1.40f;

	nav_config.turn_preview_time_s = 0.08f;

	navigation::NavigationController navigation(nav_config);
	navigation::TrackMap track_map;

	std::cout << "Initializing LiDAR...\n";

	if (!lidar.initialize()) {

		std::cerr << "LiDAR initialize failed\n";

		return 1;
	}

	if (!lidar.start()) {

		std::cerr << "LiDAR start failed\n";

		return 1;
	}

	std::cout << "Initializing OTOS...\n";

	if (!otos.initialize(1)) {

		std::cerr << "OTOS initialize failed\n";

		lidar.stop();

		return 1;
	}

	otos.setLinearUnit(kSfeOtosLinearUnitMeters);

	otos.setAngularUnit(kSfeOtosAngularUnitRadians);

	otos.resetTracking();

	otos.calibrateImu(255, true);

	std::cout << "LiDAR + OTOS ready\n";

	const std::string run_directory = logging::make_run_directory();
	logging::TelemetryLogger telemetry_log(run_directory);
	logging::WallLogger wall_log(run_directory);
	std::optional<logging::EventLogger> event_log;
	std::cout << "Logging to " << run_directory << '\n';

	cv::Mat debug_map(MAP_HEIGHT, MAP_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

	bool nav_initialized = false;

	std::uint64_t previous_timestamp_us = 0;

	navigation::NavigationMode previous_mode =
		navigation::NavigationMode::SEARCH_DIRECTION;

	std::optional<DrivingDirection> previous_direction;
	bool previous_heading_hold_active = false;

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

	while (true) {
		if (logging::stop_requested()) {
			break;
		}

		TimedLidarData scan;

		sfe_otos_pose2d_t pos{};
		sfe_otos_pose2d_t vel{};
		sfe_otos_pose2d_t acc{};

		if (!lidar.wait_for_data(scan)) {

			break;
		}

		if (scan.points.empty()) {
			continue;
		}

		const sfTkError_t otos_error = otos.getPosVelAcc(pos, vel, acc);

		if (otos_error != ksfTkErrOk) {

			std::cerr << "OTOS read error: " << static_cast<int>(otos_error)
					  << '\n';

			continue;
		}

		const float heading_rad = pos.h;

		const float speed_mps = std::hypot(vel.x, vel.y);

		if (!event_log.has_value()) {
			event_log.emplace(run_directory, scan.timestamp_us);
		}

		if (!nav_initialized) {

			navigation.reset(heading_rad);

			previous_mode = navigation.state().mode;

			nav_initialized = true;

			std::cout << "[NAV] initialized heading="
					  << heading_rad * RAD_TO_DEG << " deg\n";
		}

		const lidar::ScanMotion scan_motion{
			vel.x * std::cos(heading_rad) + vel.y * std::sin(heading_rad),
			vel.h,
			scan.scan_period_us > 0 ? scan.scan_period_us / 1'000'000.0f : 0.0f,
			scan.scan_period_us > 0};

		const auto process_start = std::chrono::steady_clock::now();

		const auto processed = lidar_processor.process(
			scan, 4, 0.035f, 0.12f, 5.0f, 0.04f, 0.10f, scan_motion);

		const navigation::MapPose map_pose{pos.x, pos.y, heading_rad};
		const auto replay_hint =
			track_map.replay_hint(map_pose, navigation.state().corner_index);

		const auto nav_result = navigation.update(
			processed, heading_rad, speed_mps, replay_hint, map_pose);

		const auto process_end = std::chrono::steady_clock::now();

		const auto process_us =
			std::chrono::duration_cast<std::chrono::microseconds>(
				process_end - process_start)
				.count();

		const auto &state = navigation.state();

		if (state.direction.has_value()) {
			track_map.set_direction(*state.direction);
		}

		if (previous_mode != navigation::NavigationMode::TURNING &&
			state.mode == navigation::NavigationMode::TURNING) {
			const float learned_trigger_m =
				nav_result.debug.effective_turn_trigger_m > 0.0f
				? nav_result.debug.effective_turn_trigger_m
				: nav_config.turn_trigger_distance_m;
			track_map.record_corner_entry(state.corner_index,
				{map_pose, learned_trigger_m, nav_config.corner_radius_m,
					nav_result.command.target_speed_mps});
			event_log->event(scan.timestamp_us, state.lap, state.corner_index,
				"corner entry learned");
			std::cout << "[MAP] learned corner " << state.corner_index
					  << " entry at (" << pos.x << ", " << pos.y << ")\n";
		}

		if (previous_mode == navigation::NavigationMode::TURNING &&
			state.mode != navigation::NavigationMode::TURNING) {
			const std::size_t completed_corner =
				(state.corner_index + navigation::TRACK_CORNER_COUNT - 1) %
				navigation::TRACK_CORNER_COUNT;
			track_map.record_corner_exit(completed_corner, map_pose);
			event_log->event(scan.timestamp_us, state.lap, completed_corner,
				"corner exit learned");
			std::cout << "[MAP] learned corner " << completed_corner
					  << " exit; map " << track_map.learned_corner_count()
					  << "/4\n";
		}

		if (state.mode != previous_mode) {

			std::cout << "\n[MODE] " << mode_to_string(previous_mode) << " -> "
					  << mode_to_string(state.mode) << '\n';
			event_log->event(scan.timestamp_us, state.lap, state.corner_index,
				std::string("MODE ") +
					logging::navigation_mode_name(previous_mode) + " -> " +
					logging::navigation_mode_name(state.mode));

			previous_mode = state.mode;
		}

		if (nav_result.debug.heading_hold_active !=
			previous_heading_hold_active) {
			event_log->fault(scan.timestamp_us,
				nav_result.debug.heading_hold_active
					? "outer wall lost, heading hold engaged"
					: "outer wall reacquired, heading hold released");
			previous_heading_hold_active = nav_result.debug.heading_hold_active;
		}

		if (state.direction != previous_direction) {

			std::cout << "[DIRECTION] " << direction_to_string(state.direction)
					  << '\n';

			previous_direction = state.direction;
		}

		std::uint64_t frame_diff_us = 0;

		if (previous_timestamp_us != 0) {

			frame_diff_us = scan.timestamp_us - previous_timestamp_us;
		}

		previous_timestamp_us = scan.timestamp_us;

		telemetry_log.record(
			logging::make_telemetry_row(scan.timestamp_us, map_pose, speed_mps,
				state, nav_result, processed.obstacles.size()));
		wall_log.record(
			processed.walls, state.mode, map_pose, scan.timestamp_us);

		debug_map.setTo(cv::Scalar(0, 0, 0));

		draw_grid(debug_map);

		draw_raw_points(debug_map, scan);

		draw_segments(debug_map, processed);

		draw_walls(debug_map, processed);

		draw_obstacles(debug_map, processed);

		draw_outer_wall(debug_map, processed, state.direction);
		draw_wall_corner(debug_map, nav_result.debug,
			nav_config.lidar_lateral_offset_m,
			nav_config.lidar_forward_offset_m);

		draw_robot(debug_map);

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
			speed_mps, nav_config.target_outer_distance_m,
			nav_config.turn_trigger_distance_m,
			nav_config.stanley.max_steering_rad, nav_config.total_turns,
			track_map, replay_hint, frame_diff_us, process_us);

		cv::imshow("NavigationController Dashboard - NO ACTUATORS", debug_map);

		const int key = cv::waitKey(1);

		if (key == 'q' || key == 'Q' || key == 27) {

			break;
		}

		if (key == 'r' || key == 'R') {
			event_log->event(scan.timestamp_us, state.lap, state.corner_index,
				"manual navigation reset");

			navigation.reset(heading_rad);
			track_map.reset();

			previous_mode = navigation.state().mode;

			previous_direction.reset();

			std::cout << "\n[NAV] RESET\n";
		}
	}

	std::cout << "\nStopping...\n";

	lidar.stop();

	cv::destroyAllWindows();

	const bool corners_ok = logging::dump_corners(run_directory, track_map);
	const bool telemetry_ok = telemetry_log.flush();
	const bool walls_ok = wall_log.flush();
	const bool events_ok = event_log.has_value() ? event_log->flush() : true;
	if (!corners_ok || !telemetry_ok || !walls_ok || !events_ok) {
		std::cerr << "Logging write failure; run data may be incomplete\n";
	}
	if (telemetry_log.dropped_row_count() > 0 ||
		wall_log.dropped_row_count() > 0) {
		std::cerr << "Logging queue overflow: telemetry="
				  << telemetry_log.dropped_row_count()
				  << " walls=" << wall_log.dropped_row_count() << '\n';
	}

	return 0;
}
