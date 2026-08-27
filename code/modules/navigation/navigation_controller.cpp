#include "navigation_controller.hpp"

#include <algorithm>
#include <cmath>

namespace navigation {

NavigationController::NavigationController(
	NavigationConfig config, InitialDirectionConfig direction_config)
	: config_(config), direction_estimator_(direction_config),
	  stanley_(config.stanley) {}

// UPDATE

NavigationResult NavigationController::update(
	const lidar::ProcessedLidarData &lidar_data, float heading_rad,
	float speed_mps) {

	NavigationResult result;

	switch (state_.mode) {

	case NavigationMode::SEARCH_DIRECTION:

		result.command =
			update_search_direction(lidar_data, heading_rad, result.debug);

		break;

	case NavigationMode::NORMAL:

		result.command =
			update_normal(lidar_data, heading_rad, speed_mps, result.debug);

		break;

	case NavigationMode::TURNING:

		result.command = update_turning(heading_rad, result.debug);

		break;

	case NavigationMode::FINISHED:

		result.command.target_speed_mps = 0.0f;

		result.command.steering_rad = 0.0f;

		break;
	}

	return result;
}

// RESET

void NavigationController::reset(float heading_rad) {

	state_ = {};

	state_.mode = NavigationMode::SEARCH_DIRECTION;

	state_.target_heading_rad = heading_rad;

	direction_estimator_.reset();
}

// SEARCH DIRECTION

NavigationCommand NavigationController::update_search_direction(
	const lidar::ProcessedLidarData &lidar_data, float heading_rad,
	NavigationDebug &debug) {

	NavigationCommand command;

	command.target_speed_mps = config_.search_speed_mps;

	command.steering_rad = calculate_search_steering(lidar_data.walls);

	// ---------------------------------------------------------
	// Estimate CW / CCW
	// ---------------------------------------------------------

	const auto direction = direction_estimator_.update(lidar_data);

	if (direction.has_value()) {

		state_.direction = *direction;

		state_.target_heading_rad = heading_rad;

		state_.mode = NavigationMode::NORMAL;

		state_.turn_armed = true;

		state_.heading_confirm_frames = 0;
	}

	debug.front_wall_valid = lidar_data.walls.front.has_value();

	return command;
}

// NORMAL

NavigationCommand NavigationController::update_normal(
	const lidar::ProcessedLidarData &lidar_data, float heading_rad,
	float speed_mps, NavigationDebug &debug) {

	NavigationCommand command;

	const TrackWalls track_walls = resolve_track_walls(lidar_data.walls);

	// ---------------------------------------------------------
	// Rearm corner trigger
	// ---------------------------------------------------------

	if (!state_.turn_armed) {

		if (!lidar_data.walls.front.has_value() ||
			lidar_data.walls.front->perpendicular_distance() >
				config_.turn_rearm_distance_m) {

			state_.turn_armed = true;
		}
	}

	// ---------------------------------------------------------
	// Start turn
	// ---------------------------------------------------------

	if (should_start_turn(lidar_data.walls)) {

		start_turn(heading_rad);

		return update_turning(heading_rad, debug);
	}

	// ---------------------------------------------------------
	// No outer wall
	// ---------------------------------------------------------

	if (track_walls.outer == nullptr) {

		command.target_speed_mps = config_.lost_wall_speed_mps;

		command.steering_rad = 0.0f;

		debug.outer_wall_valid = false;

		debug.front_wall_valid = lidar_data.walls.front.has_value();

		return command;
	}

	debug.outer_wall_valid = true;

	debug.front_wall_valid = lidar_data.walls.front.has_value();

	// ---------------------------------------------------------
	// Stanley errors
	// ---------------------------------------------------------

	const float cross_track_error_m =
		calculate_cross_track_error(*track_walls.outer);

	const float heading_error_rad =
		calculate_wall_heading_error(*track_walls.outer);

	debug.outer_distance_m = track_walls.outer->perpendicular_distance();

	debug.distance_error_m = cross_track_error_m;

	debug.wall_angle_rad = heading_error_rad;

	debug.angle_error_rad = heading_error_rad;

	// ---------------------------------------------------------
	// Stanley steering
	// ---------------------------------------------------------

	command.steering_rad =
		stanley_.calculate(cross_track_error_m, heading_error_rad, speed_mps);

	// ---------------------------------------------------------
	// Target speed
	// ---------------------------------------------------------

	command.target_speed_mps = config_.normal_speed_mps;

	if (lidar_data.walls.front.has_value()) {

		const float front_distance =
			lidar_data.walls.front->perpendicular_distance();

		if (front_distance < config_.approach_distance_m) {

			command.target_speed_mps = config_.approach_speed_mps;
		}
	}

	return command;
}

// TURNING

NavigationCommand NavigationController::update_turning(
	float heading_rad, NavigationDebug &debug) {

	NavigationCommand command;

	command.target_speed_mps = config_.turning_speed_mps;

	// ---------------------------------------------------------
	// OTOS heading error
	//
	// This value is in OTOS angular convention.
	// ---------------------------------------------------------

	const float heading_error_rad = calculate_turn_heading_error(heading_rad);

	debug.heading_error_rad = heading_error_rad;

	// ---------------------------------------------------------
	// Convert heading error to our steering convention:
	//
	// steering < 0 = LEFT
	// steering > 0 = RIGHT
	// ---------------------------------------------------------

	const float steering_error =
		heading_error_rad * config_.heading_to_steering_sign;

	command.steering_rad =
		clamp_steering(config_.turn_heading_kp * steering_error);

	// ---------------------------------------------------------
	// Check turn complete
	// ---------------------------------------------------------

	if (is_turn_complete(heading_error_rad)) {

		++state_.turn_count;

		state_.corner_index = static_cast<std::size_t>(state_.turn_count % 4);

		state_.lap = state_.turn_count / 4;

		state_.heading_confirm_frames = 0;

		if (state_.turn_count >= config_.total_turns) {

			state_.mode = NavigationMode::FINISHED;

			command.target_speed_mps = 0.0f;

			command.steering_rad = 0.0f;

			return command;
		}

		state_.mode = NavigationMode::NORMAL;

		// Do not allow immediate retrigger from
		// the wall belonging to the old corner.
		state_.turn_armed = false;

		command.steering_rad = 0.0f;

		command.target_speed_mps = config_.approach_speed_mps;
	}

	return command;
}

// INNER / OUTER

NavigationController::TrackWalls NavigationController::resolve_track_walls(
	const lidar::ResolvedWalls &walls) const {

	TrackWalls result;

	if (!state_.direction.has_value()) {
		return result;
	}

	if (*state_.direction == DrivingDirection::CLOCKWISE) {

		// CW:
		//
		// outer = LEFT
		// inner = RIGHT

		if (walls.left.has_value()) {
			result.outer = &*walls.left;
		}

		if (walls.right.has_value()) {
			result.inner = &*walls.right;
		}

	} else {

		// CCW:
		//
		// outer = RIGHT
		// inner = LEFT

		if (walls.right.has_value()) {
			result.outer = &*walls.right;
		}

		if (walls.left.has_value()) {
			result.inner = &*walls.left;
		}
	}

	return result;
}

// CROSS-TRACK ERROR

float NavigationController::calculate_cross_track_error(
	const lidar::LineSegment &outer_wall) const {

	if (!state_.direction.has_value()) {
		return 0.0f;
	}

	const float current_distance_m = outer_wall.perpendicular_distance();

	const float distance_error =
		config_.target_outer_distance_m - current_distance_m;

	// ---------------------------------------------------------
	// Steering convention:
	//
	// positive = RIGHT
	// negative = LEFT
	//
	// CW:
	// outer wall is LEFT.
	//
	// Too close to LEFT:
	// target - current > 0
	// need RIGHT
	// => positive
	//
	// CCW:
	// outer wall is RIGHT.
	//
	// Too close to RIGHT:
	// target - current > 0
	// need LEFT
	// => negative
	// ---------------------------------------------------------

	if (*state_.direction == DrivingDirection::CLOCKWISE) {

		return distance_error;
	}

	return -distance_error;
}

// WALL HEADING ERROR

float NavigationController::calculate_wall_heading_error(
	const lidar::LineSegment &outer_wall) const {

	cv::Point2f tangent = outer_wall.end - outer_wall.start;

	// Make tangent point forward (+Y).
	if (tangent.y < 0.0f) {

		tangent.x = -tangent.x;

		tangent.y = -tangent.y;
	}

	const float length = std::hypot(tangent.x, tangent.y);

	if (length < 1e-6f) {
		return 0.0f;
	}

	// Robot local coordinate:
	//
	// +Y = forward
	// +X = right
	//
	// atan2(x, y):
	//
	// +angle = path points RIGHT
	// -angle = path points LEFT
	//
	// This matches our steering convention.

	return std::atan2(tangent.x, tangent.y);
}

// TURN TRIGGER

bool NavigationController::should_start_turn(
	const lidar::ResolvedWalls &walls) const {

	if (!state_.direction.has_value()) {
		return false;
	}

	if (!state_.turn_armed) {
		return false;
	}

	if (!walls.front.has_value()) {
		return false;
	}

	return walls.front->perpendicular_distance() <=
		config_.turn_trigger_distance_m;
}

// START TURN

void NavigationController::start_turn(float heading_rad) {

	if (!state_.direction.has_value()) {
		return;
	}

	float heading_delta = 0.0f;

	if (*state_.direction == DrivingDirection::CLOCKWISE) {

		heading_delta = config_.clockwise_turn_delta_rad;

	} else {

		heading_delta = config_.counter_clockwise_turn_delta_rad;
	}

	state_.target_heading_rad = normalize_angle(heading_rad + heading_delta);

	state_.heading_confirm_frames = 0;

	state_.turn_armed = false;

	state_.mode = NavigationMode::TURNING;
}

// TURN HEADING ERROR

float NavigationController::calculate_turn_heading_error(
	float heading_rad) const {

	return normalize_angle(state_.target_heading_rad - heading_rad);
}

// TURN COMPLETE

bool NavigationController::is_turn_complete(float heading_error_rad) {

	if (std::abs(heading_error_rad) <= config_.heading_tolerance_rad) {

		++state_.heading_confirm_frames;

	} else {

		state_.heading_confirm_frames = 0;
	}

	return state_.heading_confirm_frames >= config_.heading_confirm_frames;
}

// SEARCH CENTERING

float NavigationController::calculate_search_steering(
	const lidar::ResolvedWalls &walls) const {

	if (!walls.left.has_value() || !walls.right.has_value()) {

		return 0.0f;
	}

	const float left_distance = walls.left->perpendicular_distance();

	const float right_distance = walls.right->perpendicular_distance();

	// If LEFT is closer:
	//
	// right - left > 0
	// steer RIGHT (+)

	const float center_error = right_distance - left_distance;

	return clamp_steering(config_.search_center_kp * center_error);
}

// HELPERS

float NavigationController::clamp_steering(float steering_rad) const {

	return std::clamp(steering_rad, -config_.stanley.max_steering_rad,
		config_.stanley.max_steering_rad);
}

float NavigationController::normalize_angle(float angle_rad) {

	return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

} // namespace navigation