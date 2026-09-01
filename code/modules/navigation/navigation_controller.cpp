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
	float speed_mps, const std::optional<ReplayHint> &replay_hint,
	const std::optional<MapPose> &map_pose) {

	NavigationResult result;
	const float dt_s = calculate_dt_s(lidar_data.timestamp_us);

	// Fill the wall measurement group first so every mode logs real geometry.
	// update_normal() overwrites these with its own control values, so NORMAL
	// rows keep reporting exactly what Stanley consumed.
	populate_wall_observation(lidar_data, result.debug);

	switch (state_.mode) {

	case NavigationMode::SEARCH_DIRECTION:

		result.command =
			update_search_direction(lidar_data, heading_rad, result.debug);

		break;

	case NavigationMode::NORMAL:

		result.command = update_normal(lidar_data, heading_rad, speed_mps, dt_s,
			replay_hint, map_pose, result.debug);

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
	result.debug.replay_speed_factor = calculate_replay_speed_factor();
	result.debug.active_normal_speed_mps =
		calculate_active_normal_speed_mps();
	result.debug.active_approach_speed_mps =
		calculate_active_approach_speed_mps();
	result.debug.update_dt_s = dt_s;
	result.debug.raw_update_dt_s = last_raw_update_dt_s_;
	result.debug.stanley_cross_track_term_rad =
		stanley_.last_cross_track_term_rad();
	result.debug.stanley_heading_term_rad = stanley_.last_heading_term_rad();
	result.debug.stanley_heading_integral = stanley_.heading_integral();
	result.debug.turn_heading_pid_output_rad = turn_heading_pid_.last_output();
	result.debug.turn_heading_pid_integral = turn_heading_pid_.integral();
	result.debug.turn_trigger_frames = turn_trigger_frames_;
	result.debug.turn_armed = state_.turn_armed;

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
	last_elapsed_update_s_ = 0.0f;
	last_raw_update_dt_s_ = 0.0f;

	turn_start_heading_rad_ = heading_rad;
	turn_reference_heading_rad_ = heading_rad;
	turn_reference_progress_rad_ = 0.0f;
	turn_total_angle_rad_ = 0.0f;
	turn_heading_sign_ = 0.0f;
	turn_entry_steering_rad_ = 0.0f;

	turn_trigger_frames_ = 0;
	reset_wall_corner_tracker();
	replay_speed_active_ = false;
	search_initial_center_error_m_ = 0.0f;
	search_initial_center_error_valid_ = false;
	last_valid_wall_heading_rad_ = heading_rad;
	lost_wall_timer_s_ = 0.0f;
	has_last_valid_wall_heading_ = false;

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
	if (lidar_data.walls.front.has_value()) {
		const float front_distance_m =
			lidar_data.walls.front->perpendicular_distance();
		const float slowdown_distance_m = std::max(
			config_.search_front_minimum_distance_m,
			config_.search_front_slowdown_distance_m);
		const float minimum_distance_m = std::min(
			config_.search_front_minimum_distance_m, slowdown_distance_m);
		if (std::isfinite(front_distance_m) &&
			front_distance_m < slowdown_distance_m) {
			const float distance_weight = std::clamp(
				(front_distance_m - minimum_distance_m) /
					std::max(0.01f, slowdown_distance_m - minimum_distance_m),
				0.0f, 1.0f);
			const float minimum_speed_mps = std::clamp(
				config_.search_minimum_speed_mps, 0.0f,
				std::max(0.0f, config_.search_speed_mps));
			command.target_speed_mps = minimum_speed_mps +
				(std::max(0.0f, config_.search_speed_mps) - minimum_speed_mps) *
					distance_weight;
		}
	}

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
	const std::optional<MapPose> &map_pose, NavigationDebug &debug) {

	NavigationCommand command;
	replay_speed_active_ = config_.enable_replay_speed_factors &&
		state_.lap >= 1 && replay_hint.has_value();
	if (replay_hint.has_value()) {
		debug.map_preview_valid = true;
		debug.map_distance_to_corner_m = replay_hint->distance_to_entry_m;
		debug.map_confidence = replay_hint->confidence;
	}

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

	if (should_start_turn(
			lidar_data, speed_mps, map_pose, replay_hint, debug)) {

		start_turn(heading_rad);

		return update_turning(heading_rad, speed_mps, dt_s, debug);
	}

	// ---------------------------------------------------------
	// No outer wall
	// ---------------------------------------------------------

	if (track_walls.outer == nullptr) {

		stanley_.reset();
		lost_wall_timer_s_ += last_elapsed_update_s_;

		command.target_speed_mps = config_.lost_wall_speed_mps;

		const bool heading_hold_available = has_last_valid_wall_heading_ &&
			std::isfinite(heading_rad) &&
			lost_wall_timer_s_ <= std::max(0.0f, config_.max_heading_hold_s);

		if (heading_hold_available) {
			const float heading_error_rad =
				normalize_angle(last_valid_wall_heading_rad_ - heading_rad);
			command.steering_rad = clamp_steering(
				config_.heading_to_steering_sign * heading_error_rad);
			debug.heading_tracking_error_rad = heading_error_rad;
			debug.heading_hold_active = true;
		} else {
			command.steering_rad = 0.0f;
		}

		debug.outer_wall_valid = false;

		debug.front_wall_valid = lidar_data.walls.front.has_value();
		debug.lost_wall_time_s = lost_wall_timer_s_;

		return command;
	}

	if (std::isfinite(heading_rad)) {
		last_valid_wall_heading_rad_ = heading_rad;
		has_last_valid_wall_heading_ = true;
	} else {
		has_last_valid_wall_heading_ = false;
	}
	lost_wall_timer_s_ = 0.0f;

	debug.outer_wall_valid = true;

	debug.front_wall_valid = lidar_data.walls.front.has_value();

	// ---------------------------------------------------------
	// Stanley errors
	// ---------------------------------------------------------

	const bool corridor_center_active =
		config_.follow_corridor_center && track_walls.inner != nullptr;
	const float cross_track_error_m = corridor_center_active
		? calculate_center_cross_track_error(
			  *track_walls.inner, *track_walls.outer)
		: calculate_cross_track_error(*track_walls.outer);
	const float outer_heading_error_rad =
		calculate_wall_heading_error(*track_walls.outer);
	const float heading_error_rad = corridor_center_active
		? 0.5f * (outer_heading_error_rad +
			  calculate_wall_heading_error(*track_walls.inner))
		: outer_heading_error_rad;

	debug.outer_distance_m = track_walls.outer->perpendicular_distance();
	debug.inner_distance_m = track_walls.inner != nullptr
		? track_walls.inner->perpendicular_distance()
		: 0.0f;
	debug.inner_wall_valid = track_walls.inner != nullptr;
	debug.corridor_center_active = corridor_center_active;

	debug.distance_error_m = cross_track_error_m;

	debug.wall_angle_rad = heading_error_rad;

	debug.angle_error_rad = heading_error_rad;

	// From here on distance_error_m / angle_error_rad hold live Stanley inputs
	// rather than the reporting-only observation filled in by update().
	debug.wall_following_active = true;

	// ---------------------------------------------------------
	// Stanley steering
	// ---------------------------------------------------------

	command.steering_rad = stanley_.calculate(
		cross_track_error_m, heading_error_rad, speed_mps, dt_s);

	// ---------------------------------------------------------
	// Target speed
	// ---------------------------------------------------------

	command.target_speed_mps = calculate_active_normal_speed_mps();

	if (debug.wall_corner_confirmed) {

		if (debug.wall_corner_forward_m < config_.approach_distance_m) {

			command.target_speed_mps = calculate_approach_speed_mps(
				debug.wall_corner_forward_m, debug.effective_turn_trigger_m);
		}

	} else if (lidar_data.walls.front.has_value()) {

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
			const float active_normal_speed_mps =
				calculate_active_normal_speed_mps();
			const float learned_speed_mps = std::clamp(
				replay_hint->safe_speed_mps, 0.0f, active_normal_speed_mps);
			const float preview_speed_mps = active_normal_speed_mps +
				(learned_speed_mps - active_normal_speed_mps) *
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
		std::clamp(measured_speed_mps, 0.0f, corner_speed_mps);

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
		(calculate_active_normal_speed_mps() - corner_speed_mps) *
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

// WALL OBSERVATION (reporting only)

void NavigationController::populate_wall_observation(
	const lidar::ProcessedLidarData &lidar_data, NavigationDebug &debug) const {

	debug.front_wall_valid = lidar_data.walls.front.has_value();

	// Inner/outer only exist once the driving direction is known. Before that
	// the flags stay false and the distances stay zero, which the flags make
	// unambiguous.
	const TrackWalls track_walls = resolve_track_walls(lidar_data.walls);

	debug.outer_wall_valid = track_walls.outer != nullptr;
	debug.inner_wall_valid = track_walls.inner != nullptr;

	if (track_walls.outer != nullptr) {
		debug.outer_distance_m = track_walls.outer->perpendicular_distance();
		debug.wall_angle_rad = calculate_wall_heading_error(*track_walls.outer);
	}

	if (track_walls.inner != nullptr) {
		debug.inner_distance_m = track_walls.inner->perpendicular_distance();
	}
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

float NavigationController::calculate_center_cross_track_error(
	const lidar::LineSegment &inner_wall,
	const lidar::LineSegment &outer_wall) const {
	const float inner_distance_m = inner_wall.perpendicular_distance();
	const float outer_distance_m = outer_wall.perpendicular_distance();
	const float half_distance_difference_m =
		0.5f * (inner_distance_m - outer_distance_m);

	if (*state_.direction == DrivingDirection::CLOCKWISE) {
		return half_distance_difference_m;
	}

	return -half_distance_difference_m;
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

// WALL CORNER LANDMARK

std::optional<cv::Point2f>
NavigationController::find_inner_wall_corner_candidate(
	const lidar::ProcessedLidarData &lidar_data) const {
	const TrackWalls track_walls = resolve_track_walls(lidar_data.walls);

	if (track_walls.inner == nullptr ||
		track_walls.inner->length() < config_.wall_corner_min_inner_length_m) {
		return std::nullopt;
	}

	const lidar::LineSegment &inner_wall = *track_walls.inner;
	const cv::Point2f forward_endpoint = inner_wall.start.y >= inner_wall.end.y
		? inner_wall.start
		: inner_wall.end;

	if (!std::isfinite(forward_endpoint.x) ||
		!std::isfinite(forward_endpoint.y) ||
		forward_endpoint.y < config_.wall_corner_min_forward_m ||
		forward_endpoint.y > config_.wall_corner_max_forward_m) {
		return std::nullopt;
	}

	if (has_forward_wall_continuation(
			inner_wall, forward_endpoint, lidar_data.line_segments)) {
		return std::nullopt;
	}

	return forward_endpoint;
}

bool NavigationController::has_forward_wall_continuation(
	const lidar::LineSegment &inner_wall, const cv::Point2f &forward_endpoint,
	const std::vector<lidar::LineSegment> &segments) const {
	auto angle_difference = [](float a_rad, float b_rad) {
		float difference =
			std::fmod(std::abs(a_rad - b_rad), static_cast<float>(M_PI));
		if (difference > static_cast<float>(M_PI) * 0.5f) {
			difference = static_cast<float>(M_PI) - difference;
		}
		return std::abs(difference);
	};

	auto same_segment = [](const lidar::LineSegment &a,
							const lidar::LineSegment &b) {
		constexpr float SAME_ENDPOINT_TOLERANCE_M = 0.005f;
		const bool same_order =
			cv::norm(a.start - b.start) <= SAME_ENDPOINT_TOLERANCE_M &&
			cv::norm(a.end - b.end) <= SAME_ENDPOINT_TOLERANCE_M;
		const bool reverse_order =
			cv::norm(a.start - b.end) <= SAME_ENDPOINT_TOLERANCE_M &&
			cv::norm(a.end - b.start) <= SAME_ENDPOINT_TOLERANCE_M;
		return same_order || reverse_order;
	};

	for (const lidar::LineSegment &segment : segments) {
		if (same_segment(inner_wall, segment) ||
			segment.length() < config_.wall_corner_min_inner_length_m * 0.5f) {
			continue;
		}

		if (angle_difference(inner_wall.angle_rad, segment.angle_rad) >
			config_.wall_corner_collinear_angle_rad) {
			continue;
		}

		const cv::Point2f center = (segment.start + segment.end) * 0.5f;
		const float collinear_error = std::abs(inner_wall.normal_x * center.x +
			inner_wall.normal_y * center.y + inner_wall.line_c);
		if (collinear_error > config_.wall_corner_collinear_offset_m) {
			continue;
		}

		const float furthest_forward_y =
			std::max(segment.start.y, segment.end.y);
		if (furthest_forward_y <= forward_endpoint.y + 0.04f) {
			continue;
		}

		const float gap = std::min(cv::norm(forward_endpoint - segment.start),
			cv::norm(forward_endpoint - segment.end));
		if (gap <= config_.wall_corner_continuation_gap_m) {
			return true;
		}
	}

	return false;
}

void NavigationController::update_wall_corner_landmark(
	const lidar::ProcessedLidarData &lidar_data,
	const std::optional<MapPose> &map_pose, NavigationDebug &debug) {
	const bool pose_valid = map_pose.has_value() &&
		std::isfinite(map_pose->x_m) && std::isfinite(map_pose->y_m) &&
		std::isfinite(map_pose->heading_rad);
	const auto candidate = find_inner_wall_corner_candidate(lidar_data);

	if (pose_valid && candidate.has_value()) {
		debug.wall_corner_candidate_valid = true;

		const float right_m = config_.lidar_lateral_offset_m + candidate->x;
		const float forward_m = config_.lidar_forward_offset_m + candidate->y;
		const float cosine = std::cos(map_pose->heading_rad);
		const float sine = std::sin(map_pose->heading_rad);
		const cv::Point2f candidate_world{
			map_pose->x_m - forward_m * sine + right_m * cosine,
			map_pose->y_m + forward_m * cosine + right_m * sine};

		debug.wall_corner_forward_m = forward_m;
		debug.wall_corner_lateral_m = right_m;

		if (!wall_corner_anchor_world_.has_value()) {
			wall_corner_anchor_world_ = candidate_world;
			wall_corner_filtered_world_ = candidate_world;
			wall_corner_stable_frames_ = 1;
			wall_corner_missed_frames_ = 0;
		} else if (!wall_corner_confirmed_) {
			const float stability_error =
				cv::norm(candidate_world - *wall_corner_anchor_world_);
			debug.wall_corner_stability_error_m = stability_error;

			if (stability_error <= config_.wall_corner_stability_tolerance_m) {
				++wall_corner_stable_frames_;
				const float weight =
					std::clamp(config_.wall_corner_filter_weight, 0.0f, 1.0f);
				*wall_corner_filtered_world_ +=
					(candidate_world - *wall_corner_filtered_world_) * weight;
			} else {
				wall_corner_anchor_world_ = candidate_world;
				wall_corner_filtered_world_ = candidate_world;
				wall_corner_stable_frames_ = 1;
			}
			wall_corner_missed_frames_ = 0;

			if (wall_corner_stable_frames_ >=
				std::max(1, config_.wall_corner_confirm_frames)) {
				wall_corner_confirmed_ = true;
			}
		} else {
			const float association_error =
				cv::norm(candidate_world - *wall_corner_filtered_world_);
			debug.wall_corner_stability_error_m = association_error;

			if (association_error <=
				config_.wall_corner_association_distance_m) {
				const float weight =
					std::clamp(config_.wall_corner_filter_weight, 0.0f, 1.0f);
				*wall_corner_filtered_world_ +=
					(candidate_world - *wall_corner_filtered_world_) * weight;
				wall_corner_missed_frames_ = 0;
			} else {
				++wall_corner_missed_frames_;
			}
		}
	} else {
		++wall_corner_missed_frames_;
	}

	if (wall_corner_missed_frames_ >
		std::max(0, config_.wall_corner_max_missed_frames)) {
		reset_wall_corner_tracker();
	}

	debug.wall_corner_confirm_frames = wall_corner_stable_frames_;
	debug.wall_corner_confirmed = wall_corner_confirmed_;

	if (pose_valid && wall_corner_confirmed_ &&
		wall_corner_filtered_world_.has_value()) {
		const float delta_x = wall_corner_filtered_world_->x - map_pose->x_m;
		const float delta_y = wall_corner_filtered_world_->y - map_pose->y_m;
		const float cosine = std::cos(map_pose->heading_rad);
		const float sine = std::sin(map_pose->heading_rad);
		debug.wall_corner_forward_m = -delta_x * sine + delta_y * cosine;
		debug.wall_corner_lateral_m = delta_x * cosine + delta_y * sine;
	}
}

void NavigationController::reset_wall_corner_tracker() {
	wall_corner_anchor_world_.reset();
	wall_corner_filtered_world_.reset();
	wall_corner_stable_frames_ = 0;
	wall_corner_missed_frames_ = 0;
	wall_corner_confirmed_ = false;
}

// TURN TRIGGER

bool NavigationController::should_start_turn(
	const lidar::ProcessedLidarData &lidar_data, float speed_mps,
	const std::optional<MapPose> &map_pose,
	const std::optional<ReplayHint> &replay_hint, NavigationDebug &debug) {

	// Marks effective_turn_trigger_m and the wall_corner_* group as evaluated
	// on this tick. Without it a zero in those columns is indistinguishable
	// from a tick that never ran the trigger at all (every TURNING tick).
	debug.turn_trigger_evaluated = true;

	if (!state_.direction.has_value()) {
		turn_trigger_frames_ = 0;
		return false;
	}

	if (!state_.turn_armed) {
		turn_trigger_frames_ = 0;
		return false;
	}

	const bool geometry_available = config_.use_wall_corner_trigger &&
		map_pose.has_value() && std::isfinite(map_pose->x_m) &&
		std::isfinite(map_pose->y_m) && std::isfinite(map_pose->heading_rad);

	if (geometry_available) {
		update_wall_corner_landmark(lidar_data, map_pose, debug);
	}

	bool trigger_condition = false;
	if (geometry_available && debug.wall_corner_confirmed) {
		debug.turn_trigger_source = TurnTriggerSource::INNER_CORNER;
		debug.effective_turn_trigger_m =
			calculate_geometric_turn_trigger_m(speed_mps);
		trigger_condition =
			debug.wall_corner_forward_m <= debug.effective_turn_trigger_m;
		debug.wall_corner_trigger_active = trigger_condition;
	} else {
		debug.turn_trigger_source = geometry_available
			? TurnTriggerSource::FRONT_FALLBACK
			: TurnTriggerSource::LEGACY_FRONT;
		debug.effective_turn_trigger_m = geometry_available
			? calculate_front_wall_fallback_trigger_m(speed_mps)
			: calculate_effective_turn_trigger_m(speed_mps);
		debug.front_wall_fallback_active = geometry_available;
		trigger_condition = lidar_data.walls.front.has_value() &&
			lidar_data.walls.front->perpendicular_distance() <=
				debug.effective_turn_trigger_m;
	}

	if (state_.lap >= 1 && replay_hint.has_value() &&
		replay_hint->distance_to_entry_m >
			std::max(0.0f, config_.replay_turn_gate_distance_m)) {
		const bool safety_override = lidar_data.walls.front.has_value() &&
			lidar_data.walls.front->perpendicular_distance() <=
				std::max(
					0.0f, config_.replay_front_safety_override_distance_m);
		if (!safety_override) {
			debug.replay_gate_suppressed = trigger_condition;
			trigger_condition = false;
			debug.wall_corner_trigger_active = false;
			debug.front_wall_fallback_active = false;
		}
	}

	if (trigger_condition) {
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
	reset_wall_corner_tracker();
	lost_wall_timer_s_ = 0.0f;
	has_last_valid_wall_heading_ = false;

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

	const bool heading_confirmed = state_.heading_confirm_frames >=
		std::max(1, config_.heading_confirm_frames);
	const float signed_remaining = turn_heading_sign_ * heading_error_rad;
	const bool crossed_target = signed_remaining <= 0.0f &&
		turn_reference_progress_rad_ >= turn_total_angle_rad_ * 0.80f;

	return heading_confirmed || crossed_target;
}

// SPEED PROFILE

float NavigationController::calculate_corner_speed_mps() const {
	const float requested_speed_mps = std::max(0.0f, config_.turning_speed_mps);
	const float maximum_speed_mps = std::min(
		requested_speed_mps, std::max(0.0f, config_.normal_speed_mps));

	if (config_.max_lateral_acceleration_mps2 <= 0.0f) {
		return maximum_speed_mps;
	}

	const float radius_m = std::max(0.05f, config_.corner_radius_m);
	const float lateral_limit_mps =
		std::sqrt(config_.max_lateral_acceleration_mps2 * radius_m);

	return std::min(maximum_speed_mps, lateral_limit_mps);
}

float NavigationController::calculate_replay_speed_factor() const {
	if (!replay_speed_active_) {
		return 1.0f;
	}

	if (state_.lap >= 2) {
		return std::max(1.0f, config_.lap3_speed_factor);
	}

	return std::max(1.0f, config_.lap2_speed_factor);
}

float NavigationController::calculate_active_normal_speed_mps() const {
	const float base_speed_mps = std::max(0.0f, config_.normal_speed_mps);
	if (!replay_speed_active_) {
		return base_speed_mps;
	}

	return std::min(base_speed_mps * calculate_replay_speed_factor(),
		std::max(base_speed_mps, config_.maximum_replay_speed_mps));
}

float NavigationController::calculate_active_approach_speed_mps() const {
	const float base_speed_mps = std::max(0.0f, config_.approach_speed_mps);
	if (!replay_speed_active_) {
		return base_speed_mps;
	}

	const float factor_weight =
		std::clamp(config_.replay_approach_factor_weight, 0.0f, 1.0f);
	const float approach_factor = 1.0f +
		(calculate_replay_speed_factor() - 1.0f) * factor_weight;
	return std::min(base_speed_mps * approach_factor,
		std::max(base_speed_mps, config_.maximum_replay_speed_mps));
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

float NavigationController::calculate_geometric_turn_trigger_m(
	float speed_mps) const {
	const float safe_speed_mps =
		std::isfinite(speed_mps) ? std::abs(speed_mps) : 0.0f;
	const float radius_m = std::max(0.05f, config_.corner_radius_m);
	const float preview_m =
		safe_speed_mps * std::max(0.0f, config_.turn_preview_time_s);
	return std::max(
		0.05f, radius_m + config_.wall_corner_to_path_offset_m + preview_m);
}

float NavigationController::calculate_front_wall_fallback_trigger_m(
	float speed_mps) const {
	const float safe_speed_mps =
		std::isfinite(speed_mps) ? std::abs(speed_mps) : 0.0f;
	const float trigger_m =
		std::max(0.05f, config_.front_wall_fallback_distance_m) +
		safe_speed_mps * std::max(0.0f, config_.turn_preview_time_s);
	return std::min(
		trigger_m, std::max(0.05f, config_.approach_distance_m - 0.05f));
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
		const float normal_speed_mps = calculate_active_normal_speed_mps();
		return normal_speed_mps +
			(calculate_active_approach_speed_mps() - normal_speed_mps) * weight;
	}

	const float weight = smoothstep((progress - 0.5f) * 2.0f);

	const float approach_speed_mps = calculate_active_approach_speed_mps();
	return approach_speed_mps +
		(corner_speed_mps - approach_speed_mps) * weight;
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
	last_elapsed_update_s_ = std::max(0.0f, dt_s);

	last_raw_update_dt_s_ = dt_s;

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
	const lidar::ResolvedWalls &walls) {

	if (!walls.left.has_value() || !walls.right.has_value()) {

		return 0.0f;
	}

	const float left_distance = walls.left->perpendicular_distance();

	const float right_distance = walls.right->perpendicular_distance();

	// If LEFT is closer:
	//
	// right - left > 0
	// steer RIGHT (+)

	const float measured_center_error = right_distance - left_distance;
	if (config_.search_preserve_initial_offset &&
		!search_initial_center_error_valid_) {
		search_initial_center_error_m_ = measured_center_error;
		search_initial_center_error_valid_ = true;
	}
	const float center_error = config_.search_preserve_initial_offset
		? measured_center_error - search_initial_center_error_m_
		: measured_center_error;

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
