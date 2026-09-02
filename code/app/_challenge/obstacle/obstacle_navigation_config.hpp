#pragma once

#include "navigation_controller.hpp"

namespace obstacle_challenge {

// Obstacle navigation owns this baseline. It is intentionally copied from
// open_challenge::make_navigation_config() first so both challenges start with
// the same measured vehicle model and wall-following behaviour. Tune this
// function independently after obstacle runs; obstacle avoidance is composed
// on top of navigation in ObstacleController.
inline navigation::NavigationConfig make_navigation_config() {
	navigation::NavigationConfig config;
	constexpr float PI = 3.14159265358979323846f;

	config.follow_corridor_center = true;
	config.search_preserve_initial_offset = true;

	config.search_speed_mps = 0.09f;
	config.search_minimum_speed_mps = 0.06f;
	config.normal_speed_mps = 0.40f;
	config.approach_speed_mps = 0.25f;
	config.turning_speed_mps = 0.20f;
	config.maximum_replay_speed_mps = 0.40f;
	config.max_acceleration_mps2 = 3.0f;
	config.max_deceleration_mps2 = 5.0f;
	config.max_lateral_acceleration_mps2 = 1.097f;
	config.curvature_gain = 1.3f;

	config.search_front_slowdown_distance_m = 0.70f;
	config.search_front_minimum_distance_m = 0.20f;
	config.target_outer_distance_m = 0.27f;

	config.lap2_speed_factor = 1.10f;
	config.lap3_speed_factor = 1.10f;
	config.replay_approach_factor_weight = 0.50f;
	config.replay_turn_gate_distance_m = 0.40f;
	config.replay_front_safety_override_distance_m = 0.25f;

	config.approach_distance_m = 0.95f;
	config.turn_trigger_distance_m = 0.90f;
	config.turn_rearm_distance_m = 0.40f;
	config.turn_preview_time_s = 0.15f;
	config.use_wall_corner_trigger = false;
	config.front_wall_fallback_distance_m = 0.80f;
	config.wall_corner_to_path_offset_m = 0.02f;
	config.corner_radius_m = 0.315f;

	config.turn_entry_blend_rad = 12.5f * PI / 180.0f;
	config.max_steering_rate_rad_s = 10.0f;
	config.turn_exit_blend_rad = 20.0f * PI / 180.0f;

	config.heading_tolerance_rad = 10.50f * PI / 180.0f;
	config.exit_acceleration_blend_rad = 20.0f * PI / 180.0f;
	config.heading_confirm_frames = 2;

	config.stanley.k = 1.05f;
	config.stanley.max_steering_rad = 42.0f * PI / 180.0f;
	config.max_steering_rad = 42.0f * PI / 180.0f;
	config.servo_min_pulse_us = 850;
	config.servo_center_pulse_us = 1400;
	config.servo_max_pulse_us = 1950;
	config.maximum_servo_step_us = 350;
	config.stanley.heading_pid.kp = 0.75f;
	config.stanley.heading_pid.ki = 0.05f;
	config.stanley.heading_pid.kd = 0.085f;

	config.turn_heading_pid.kp = 0.0f;
	config.turn_heading_pid.ki = 0.0f;
	config.turn_heading_pid.kd = 0.0f;

	config.total_turns = 12;
	config.wall_corner_confirm_frames = 2;
	config.wall_corner_stability_tolerance_m = 0.04f;
	config.wall_corner_min_inner_length_m = 0.20f;
	return config;
}

} // namespace obstacle_challenge
