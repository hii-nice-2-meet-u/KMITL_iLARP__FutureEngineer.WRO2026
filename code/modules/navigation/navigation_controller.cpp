#include "navigation_controller.hpp"

#include <algorithm>
#include <cmath>

namespace navigation {

NavigationController::NavigationController(
	NavigationConfig config, InitialDirectionConfig direction_config)
	: config_(config), direction_estimator_(direction_config),
	  stanley_(config.stanley), turn_heading_pid_(config.turn_heading_pid),
	  speed_pid_(config.speed_pid) {}

// UPDATE

NavigationResult NavigationController::update(
	const lidar::ProcessedLidarData &lidar_data, float heading_rad,
	float speed_mps, const std::optional<ReplayHint> &replay_hint) {

	NavigationResult result;
	const float dt_s = calculate_dt_s(lidar_data.timestamp_us);

	switch (state_.mode) {

	case NavigationMode::SEARCH_DIRECTION:

		result.command =
			update_search_direction(lidar_data, heading_rad, result.debug);

		break;

	case NavigationMode::NORMAL:

		result.command = update_normal(lidar_data, heading_rad, speed_mps, dt_s,
			replay_hint, result.debug);

		break;

	case NavigationMode::TURNING:

		result.command =
			update_turning(heading_rad, speed_mps, dt_s, result.debug);

		break;

	case NavigationMode::FINISHED:

		result.command.target_speed_mps = 0.0f;

		result.command.steering_rad = 0.0f;

		break;
	}

	result.debug.raw_steering_rad = result.command.steering_rad;
	result.debug.raw_target_speed_mps = result.command.target_speed_mps;
	result.debug.corner_speed_mps = calculate_corner_speed_mps();
	result.debug.update_dt_s = dt_s;

	result.command = condition_command(result.command, speed_mps, dt_s,
		state_.mode == NavigationMode::FINISHED);
	result.debug.target_acceleration_mps2 =
		result.command.target_acceleration_mps2;

	return result;
}

// RESET

void NavigationController::reset(float heading_rad) {

	state_ = {};

	state_.mode = NavigationMode::SEARCH_DIRECTION;

	state_.target_heading_rad = heading_rad;

	direction_estimator_.reset();

	previous_timestamp_us_ = 0;

	turn_start_heading_rad_ = heading_rad;
	turn_reference_heading_rad_ = heading_rad;
	turn_reference_progress_rad_ = 0.0f;
	turn_total_angle_rad_ = 0.0f;
	turn_heading_sign_ = 0.0f;
	turn_entry_steering_rad_ = 0.0f;

	turn_trigger_frames_ = 0;

	conditioned_steering_rad_ = 0.0f;
	conditioned_speed_mps_ = 0.0f;
	command_conditioner_initialized_ = false;

	stanley_.reset();
	turn_heading_pid_.reset();
	speed_pid_.reset();
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
	float speed_mps, float dt_s, const std::optional<ReplayHint> &replay_hint,
	NavigationDebug &debug) {

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
			turn_trigger_frames_ = 0;
		}
	}

	// ---------------------------------------------------------
	// Start turn
	// ---------------------------------------------------------

	if (should_start_turn(lidar_data.walls, speed_mps, debug)) {

		start_turn(heading_rad);

		return update_turning(heading_rad, speed_mps, dt_s, debug);
	}

	// ---------------------------------------------------------
	// No outer wall
	// ---------------------------------------------------------

	if (track_walls.outer == nullptr) {

		stanley_.reset();

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

	command.steering_rad = stanley_.calculate(
		cross_track_error_m, heading_error_rad, speed_mps, dt_s);

	// ---------------------------------------------------------
	// Target speed
	// ---------------------------------------------------------

	command.target_speed_mps = config_.normal_speed_mps;

	if (lidar_data.walls.front.has_value()) {

		const float front_distance =
			lidar_data.walls.front->perpendicular_distance();

		if (front_distance < config_.approach_distance_m) {

			const float effective_trigger =
				calculate_effective_turn_trigger_m(speed_mps);

			command.target_speed_mps =
				calculate_approach_speed_mps(front_distance, effective_trigger);
		}
	}

	// A learned map is a preview source only. It may start the speed transition
	// earlier on laps two and three, but the live front wall still owns the
	// NORMAL -> TURNING transition.
	if (replay_hint.has_value()) {
		debug.map_preview_valid = true;
		debug.map_distance_to_corner_m = replay_hint->distance_to_entry_m;
		debug.map_confidence = replay_hint->confidence;

		if (replay_hint->approach_recommended) {
			debug.map_approach_active = true;
			const float preview_distance_m =
				std::max(0.10f, config_.approach_distance_m);
			const float progress = 1.0f -
				std::clamp(
					replay_hint->distance_to_entry_m / preview_distance_m, 0.0f,
					1.0f);
			const float learned_speed_mps = std::clamp(
				replay_hint->safe_speed_mps, 0.0f, config_.normal_speed_mps);
			const float preview_speed_mps = config_.normal_speed_mps +
				(learned_speed_mps - config_.normal_speed_mps) *
					smoothstep(progress);
			command.target_speed_mps =
				std::min(command.target_speed_mps, preview_speed_mps);
		}
	}

	return command;
}

// TURNING

NavigationCommand NavigationController::update_turning(
	float heading_rad, float speed_mps, float dt_s, NavigationDebug &debug) {

	NavigationCommand command;

	const float radius_m = std::max(0.05f, config_.corner_radius_m);
	const float corner_speed_mps = calculate_corner_speed_mps();

	const float measured_speed_mps =
		std::isfinite(speed_mps) ? std::abs(speed_mps) : 0.0f;
	const float reference_speed_mps =
		std::max(measured_speed_mps, std::min(corner_speed_mps, 0.25f));

	const float remaining_reference_rad =
		std::max(0.0f, turn_total_angle_rad_ - turn_reference_progress_rad_);

	const float reference_step_rad = std::min(
		remaining_reference_rad, reference_speed_mps / radius_m * dt_s);

	turn_reference_progress_rad_ += reference_step_rad;

	turn_reference_heading_rad_ = normalize_angle(turn_start_heading_rad_ +
		turn_heading_sign_ * turn_reference_progress_rad_);

	// ---------------------------------------------------------
	// OTOS heading error
	//
	// This value is in OTOS angular convention.
	// ---------------------------------------------------------

	const float heading_error_rad = calculate_turn_heading_error(heading_rad);
	const float tracking_error_rad =
		normalize_angle(turn_reference_heading_rad_ - heading_rad);

	debug.heading_error_rad = heading_error_rad;
	debug.heading_tracking_error_rad = tracking_error_rad;
	debug.turn_reference_heading_rad = turn_reference_heading_rad_;
	debug.turn_progress = turn_total_angle_rad_ > 1e-6f
		? std::clamp(
			  turn_reference_progress_rad_ / turn_total_angle_rad_, 0.0f, 1.0f)
		: 1.0f;

	// ---------------------------------------------------------
	// Convert heading error to our steering convention:
	//
	// steering < 0 = LEFT
	// steering > 0 = RIGHT
	// ---------------------------------------------------------

	const float entry_weight = config_.turn_entry_blend_rad > 1e-6f
		? smoothstep(
			  turn_reference_progress_rad_ / config_.turn_entry_blend_rad)
		: 1.0f;

	const float remaining_after_step_rad =
		std::max(0.0f, turn_total_angle_rad_ - turn_reference_progress_rad_);

	const float exit_weight = config_.turn_exit_blend_rad > 1e-6f
		? smoothstep(remaining_after_step_rad / config_.turn_exit_blend_rad)
		: 1.0f;

	const float feedforward_magnitude_rad =
		std::atan2(std::max(0.01f, config_.wheelbase_m), radius_m);

	const float feedforward_rad = config_.heading_to_steering_sign *
		turn_heading_sign_ * feedforward_magnitude_rad *
		std::min(entry_weight, exit_weight);

	const float entry_steering_rad =
		turn_entry_steering_rad_ * (1.0f - entry_weight);

	const float tracking_steering_rad =
		turn_heading_pid_.calculate(0.0f, -tracking_error_rad, dt_s) *
		config_.heading_to_steering_sign;

	debug.turn_feedforward_rad = feedforward_rad;

	command.steering_rad = clamp_steering(
		entry_steering_rad + feedforward_rad + tracking_steering_rad);

	const float exit_acceleration_weight =
		config_.exit_acceleration_blend_rad > 1e-6f ? 1.0f -
			smoothstep(std::abs(heading_error_rad) /
				config_.exit_acceleration_blend_rad)
													: 0.0f;

	command.target_speed_mps = corner_speed_mps +
		(config_.normal_speed_mps - corner_speed_mps) *
			exit_acceleration_weight;

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
		turn_trigger_frames_ = 0;
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

bool NavigationController::should_start_turn(const lidar::ResolvedWalls &walls,
	float speed_mps, NavigationDebug &debug) {

	if (!state_.direction.has_value()) {
		turn_trigger_frames_ = 0;
		return false;
	}

	if (!state_.turn_armed) {
		turn_trigger_frames_ = 0;
		return false;
	}

	if (!walls.front.has_value()) {
		turn_trigger_frames_ = 0;
		return false;
	}

	const float effective_trigger_m =
		calculate_effective_turn_trigger_m(speed_mps);

	debug.effective_turn_trigger_m = effective_trigger_m;

	if (walls.front->perpendicular_distance() <= effective_trigger_m) {
		++turn_trigger_frames_;
	} else {
		turn_trigger_frames_ = 0;
	}

	return turn_trigger_frames_ >=
		std::max(1, config_.turn_trigger_confirm_frames);
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

	// Advance from the previous straight's cardinal reference, not from the
	// instantaneous (possibly noisy or slightly misaligned) OTOS heading. This
	// prevents a few degrees of error from accumulating at every corner.
	state_.target_heading_rad =
		normalize_angle(state_.target_heading_rad + heading_delta);

	float signed_turn_angle_rad =
		normalize_angle(state_.target_heading_rad - heading_rad);

	// Keep the commanded driving direction even close to the +/-pi wrap point.
	if (heading_delta < 0.0f && signed_turn_angle_rad > 0.0f) {
		signed_turn_angle_rad -= 2.0f * static_cast<float>(M_PI);
	} else if (heading_delta > 0.0f && signed_turn_angle_rad < 0.0f) {
		signed_turn_angle_rad += 2.0f * static_cast<float>(M_PI);
	}

	turn_start_heading_rad_ = heading_rad;
	turn_reference_heading_rad_ = heading_rad;
	turn_reference_progress_rad_ = 0.0f;
	turn_total_angle_rad_ = std::abs(signed_turn_angle_rad);
	turn_heading_sign_ = heading_delta >= 0.0f ? 1.0f : -1.0f;
	turn_entry_steering_rad_ = conditioned_steering_rad_;
	turn_heading_pid_.reset();

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
	const bool reference_complete =
		turn_reference_progress_rad_ >= turn_total_angle_rad_ - 1e-5f;

	if (reference_complete &&
		std::abs(heading_error_rad) <= config_.heading_tolerance_rad) {

		++state_.heading_confirm_frames;

	} else {

		state_.heading_confirm_frames = 0;
	}

	return state_.heading_confirm_frames >=
		std::max(1, config_.heading_confirm_frames);
}

// SPEED PROFILE

float NavigationController::calculate_corner_speed_mps() const {
	const float requested_speed_mps = std::max(0.0f, config_.turning_speed_mps);
	const float maximum_speed_mps =
		std::min(requested_speed_mps, std::max(0.0f, config_.normal_speed_mps));

	if (config_.max_lateral_acceleration_mps2 <= 0.0f) {
		return maximum_speed_mps;
	}

	const float radius_m = std::max(0.05f, config_.corner_radius_m);
	const float lateral_limit_mps =
		std::sqrt(config_.max_lateral_acceleration_mps2 * radius_m);

	return std::min(maximum_speed_mps, lateral_limit_mps);
}

float NavigationController::calculate_effective_turn_trigger_m(
	float speed_mps) const {
	const float safe_speed_mps =
		std::isfinite(speed_mps) ? std::abs(speed_mps) : 0.0f;
	const float preview_trigger_m = config_.turn_trigger_distance_m +
		safe_speed_mps * std::max(0.0f, config_.turn_preview_time_s);
	const float latest_smooth_entry_m = std::max(
		config_.turn_trigger_distance_m, config_.approach_distance_m - 0.05f);

	return std::clamp(preview_trigger_m,
		std::max(0.0f, config_.turn_trigger_distance_m),
		std::max(0.0f, latest_smooth_entry_m));
}

float NavigationController::calculate_approach_speed_mps(
	float front_distance_m, float effective_trigger_m) const {
	const float corner_speed_mps = calculate_corner_speed_mps();
	const float approach_distance_m =
		std::max(config_.approach_distance_m, effective_trigger_m + 1e-3f);

	const float progress = 1.0f -
		std::clamp((front_distance_m - effective_trigger_m) /
				(approach_distance_m - effective_trigger_m),
			0.0f, 1.0f);

	if (progress < 0.5f) {
		const float weight = smoothstep(progress * 2.0f);
		return config_.normal_speed_mps +
			(config_.approach_speed_mps - config_.normal_speed_mps) * weight;
	}

	const float weight = smoothstep((progress - 0.5f) * 2.0f);

	return config_.approach_speed_mps +
		(corner_speed_mps - config_.approach_speed_mps) * weight;
}

// COMMAND CONDITIONING

float NavigationController::calculate_dt_s(std::uint64_t timestamp_us) {
	float dt_s = config_.nominal_update_period_s;

	if (timestamp_us != 0 && previous_timestamp_us_ != 0 &&
		timestamp_us > previous_timestamp_us_) {
		dt_s =
			static_cast<float>(timestamp_us - previous_timestamp_us_) * 1e-6f;
	}

	if (timestamp_us != 0) {
		previous_timestamp_us_ = timestamp_us;
	}

	const float min_dt_s = std::max(1e-4f, config_.min_update_period_s);
	const float max_dt_s = std::max(min_dt_s, config_.max_update_period_s);

	return std::clamp(dt_s, min_dt_s, max_dt_s);
}

NavigationCommand NavigationController::condition_command(
	const NavigationCommand &command, float measured_speed_mps, float dt_s,
	bool stop_immediately) {
	if (stop_immediately) {
		conditioned_speed_mps_ = 0.0f;
		conditioned_steering_rad_ = 0.0f;
		command_conditioner_initialized_ = true;
		speed_pid_.reset();
		const float acceleration_mps2 = speed_pid_.calculate(0.0f,
			std::isfinite(measured_speed_mps) ? std::abs(measured_speed_mps)
											  : 0.0f,
			dt_s);
		return {0.0f, 0.0f, acceleration_mps2};
	}

	if (!command_conditioner_initialized_) {
		conditioned_speed_mps_ = 0.0f;
		conditioned_steering_rad_ = 0.0f;
		command_conditioner_initialized_ = true;
	}

	const float requested_speed_mps = std::isfinite(command.target_speed_mps)
		? std::max(0.0f, command.target_speed_mps)
		: 0.0f;
	const float speed_delta_mps = requested_speed_mps - conditioned_speed_mps_;
	const float speed_rate_mps2 = speed_delta_mps >= 0.0f
		? std::max(0.0f, config_.max_acceleration_mps2)
		: std::max(0.0f, config_.max_deceleration_mps2);
	const float max_speed_delta_mps = speed_rate_mps2 * dt_s;

	conditioned_speed_mps_ +=
		std::clamp(speed_delta_mps, -max_speed_delta_mps, max_speed_delta_mps);

	const float requested_steering_rad = std::isfinite(command.steering_rad)
		? clamp_steering(command.steering_rad)
		: 0.0f;
	const float time_constant_s =
		std::max(0.0f, config_.steering_filter_time_constant_s);
	const float filter_weight =
		time_constant_s <= 1e-6f ? 1.0f : dt_s / (time_constant_s + dt_s);
	const float filtered_target_rad = conditioned_steering_rad_ +
		(requested_steering_rad - conditioned_steering_rad_) * filter_weight;
	const float max_steering_delta_rad =
		std::max(0.0f, config_.max_steering_rate_rad_s) * dt_s;

	conditioned_steering_rad_ +=
		std::clamp(filtered_target_rad - conditioned_steering_rad_,
			-max_steering_delta_rad, max_steering_delta_rad);
	conditioned_steering_rad_ = clamp_steering(conditioned_steering_rad_);

	const float actual_speed_mps =
		std::isfinite(measured_speed_mps) ? std::abs(measured_speed_mps) : 0.0f;
	const float acceleration_mps2 =
		speed_pid_.calculate(conditioned_speed_mps_, actual_speed_mps, dt_s);

	return {
		conditioned_speed_mps_, conditioned_steering_rad_, acceleration_mps2};
}

float NavigationController::smoothstep(float value) {
	const float x = std::clamp(value, 0.0f, 1.0f);
	return x * x * (3.0f - 2.0f * x);
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
