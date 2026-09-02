#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>

#include "lidar_module.hpp"
#include "lidar_processor.hpp"
#include "navigation_controller.hpp"
#include "otos.hpp"

namespace open_challenge {

constexpr float PI = 3.14159265358979323846f;
constexpr float RAD_TO_DEG = 180.0f / PI;

inline float normalize_angle(float angle_rad) {
	return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

inline bool calibrate_otos(otos::OTOS &otos_device) {
	std::cout << "Keep robot stationary: calibrating OTOS IMU...\n";
	const sfTkError_t calibration_error = otos_device.calibrateImu(255, true);
	if (calibration_error != ksfTkErrOk) {
		std::cerr << "OTOS IMU calibration failed: "
				  << static_cast<int>(calibration_error) << '\n';
		return false;
	}
	const sfTkError_t reset_error = otos_device.resetTracking();
	if (reset_error != ksfTkErrOk) {
		std::cerr << "OTOS tracking reset failed after calibration: "
				  << static_cast<int>(reset_error) << '\n';
		return false;
	}
	std::cout << "OTOS IMU calibration complete; tracking reset\n";
	return true;
}

struct OtosScalars {
	std::optional<float> linear;
	std::optional<float> angular;
};

inline OtosScalars read_otos_scalars(otos::OTOS &otos_device) {
	OtosScalars scalars;
	float value = 0.0f;
	if (otos_device.getLinearScalar(value) == ksfTkErrOk) {
		scalars.linear = value;
	}
	if (otos_device.getAngularScalar(value) == ksfTkErrOk) {
		scalars.angular = value;
	}
	return scalars;
}

inline navigation::NavigationConfig make_navigation_config() {
	navigation::NavigationConfig config;

	// ======
	// Wall following and initial alignment
	config.follow_corridor_center = true;
	config.search_preserve_initial_offset = true;

	// ======
	// Speed and acceleration
	config.search_speed_mps = 0.19f;
	config.search_minimum_speed_mps = 0.06f;
	config.normal_speed_mps = 0.45f;
	config.approach_speed_mps = 0.26f;
	config.turning_speed_mps = 0.28f;
	config.maximum_replay_speed_mps = 0.55f;
	config.max_acceleration_mps2 = 5.0f;
	config.max_deceleration_mps2 = 5.0f;
	config.max_lateral_acceleration_mps2 = 0.50f;

	// ======
	// Direction search and corridor target
	config.search_front_slowdown_distance_m = 0.70f;
	config.search_front_minimum_distance_m = 0.20f;
	config.target_outer_distance_m = 0.27f;

	// ======
	// Learned-map replay for laps 2 and 3
	config.lap2_speed_factor = 1.10f;
	config.lap3_speed_factor = 1.15f;
	config.replay_approach_factor_weight = 0.50f;
	config.replay_turn_gate_distance_m = 0.40f;
	config.replay_front_safety_override_distance_m = 0.25f;

	// ======
	// Turn approach and trigger geometry
	config.approach_distance_m = 0.90f;
	config.turn_trigger_distance_m = 0.65f;
	config.turn_rearm_distance_m = 0.80f;
	config.turn_preview_time_s = 0.1f;
	config.use_wall_corner_trigger = true;
	config.front_wall_fallback_distance_m = 0.56f;
	config.wall_corner_to_path_offset_m = 0.02f;
	// Must stay above the vehicle's minimum turning radius. With
	// wheelbase_m = 0.16375 that is 0.210 m at the 38 deg steering clamp and
	// 0.164 m at the 45 deg actuator limit. This value also sets the corner
	// feed-forward magnitude atan2(wheelbase_m, radius) and the geometric turn
	// trigger distance, so it cannot be tuned for cornering line alone.
	config.corner_radius_m = 0.45f;

	// ======
	// Steering transition and rate limit
	config.turn_entry_blend_rad = 22.5f * PI / 180.0f;
	config.max_steering_rate_rad_s = 3.0f;
	config.turn_exit_blend_rad = 32.0f * PI / 180.0f;

	// ======
	// Turn completion
	config.heading_tolerance_rad = 16.50f * PI / 180.0f;
	config.exit_acceleration_blend_rad = 20.0f * PI / 180.0f;
	config.heading_confirm_frames = 2;

	// ======
	// Corridor-following steering controller
	config.stanley.k = 1.00f;
	config.stanley.max_steering_rad = 38.0f * PI / 180.0f;
	config.stanley.heading_pid.kp = 1.00f;
	config.stanley.heading_pid.ki = 0.08f;
	config.stanley.heading_pid.kd = 0.075f;

	// ======
	// Turn heading controller
	config.turn_heading_pid.kp = 0.30f;
	config.turn_heading_pid.ki = 0.08f;
	config.turn_heading_pid.kd = 0.048f;

	// ======
	// Course completion and inner-corner validation
	config.total_turns = 12;
	config.wall_corner_confirm_frames = 2;
	config.wall_corner_stability_tolerance_m = 0.04f;
	config.wall_corner_min_inner_length_m = 0.20f;
	return config;
}

inline lidar::ProcessedLidarData process_scan(lidar::LidarProcessor &processor,
	const TimedLidarData &scan, float wall_correction_rad) {
	return processor.process(
		scan, wall_correction_rad, 4, 0.035f, 0.12f, 5.0f, 0.04f, 0.10f);
}

inline const char *mode_name(navigation::NavigationMode mode) {
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

inline const char *direction_name(
	const std::optional<DrivingDirection> &direction) {
	if (!direction.has_value()) {
		return "UNKNOWN";
	}
	return *direction == DrivingDirection::CLOCKWISE ? "CW" : "CCW";
}

inline void print_command(const navigation::NavigationResult &result,
	const navigation::NavigationState &state, float heading_rad,
	float measured_speed_mps, std::int16_t wheel_rpm,
	std::uint16_t servo_pulse_us) {
	std::cout << "[NAV] mode=" << mode_name(state.mode)
			  << " dir=" << direction_name(state.direction)
			  << " turn=" << state.turn_count << "/12"
			  << " heading=" << heading_rad * RAD_TO_DEG << "deg"
			  << " speed=" << measured_speed_mps << "m/s"
			  << " target=" << result.command.target_speed_mps << "m/s"
			  << " steering=" << result.command.steering_rad * RAD_TO_DEG
			  << "deg wheel_rpm=" << wheel_rpm << " pulse=" << servo_pulse_us
			  << "us\n";
}

} // namespace open_challenge
