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

inline navigation::NavigationConfig make_navigation_config() {
	navigation::NavigationConfig config;
	config.target_outer_distance_m = 0.30f;
	config.normal_speed_mps = 0.85f;
	config.turning_speed_mps = 0.65f;
	config.approach_distance_m = 0.90f;

	config.stanley.k = 0.85f;
	config.stanley.max_steering_rad = 45.0f * PI / 180.0f;
	config.stanley.heading_pid.kp = 1.00f;
	config.stanley.heading_pid.ki = 0.12f;
	config.stanley.heading_pid.kd = 0.025f;

	config.turn_heading_pid.kp = 0.80f;
	config.turn_heading_pid.ki = 0.08f;
	config.turn_heading_pid.kd = 0.020f;

	config.speed_pid.kp = 4.0f;
	config.speed_pid.ki = 1.0f;
	config.speed_pid.kd = 0.04f;

	config.total_turns = 12;
	return config;
}

inline lidar::ProcessedLidarData process_scan(lidar::LidarProcessor &processor,
	const TimedLidarData &scan, float wall_correction_rad) {
	return processor.process(scan, wall_correction_rad,
		4,		// min_segment_point
		0.035f, // max_line_error_m
		0.12f,	// max_point_gap_m
		5.0f,	// max_angle_diff deg
		0.04f,	// max_collinear_error_m
		0.10f	// max_segment_gap_m
	);
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
	float measured_speed_mps, std::int16_t power_percent,
	std::uint16_t servo_pulse_us) {
	std::cout << "[NAV] mode=" << mode_name(state.mode)
			  << " dir=" << direction_name(state.direction)
			  << " turn=" << state.turn_count << "/12"
			  << " heading=" << heading_rad * RAD_TO_DEG << "deg"
			  << " speed=" << measured_speed_mps << "m/s"
			  << " target=" << result.command.target_speed_mps << "m/s"
			  << " accel=" << result.command.target_acceleration_mps2 << "m/s2"
			  << " steering=" << result.command.steering_rad * RAD_TO_DEG
			  << "deg power=" << power_percent << "% pulse=" << servo_pulse_us
			  << "us\n";
}

} // namespace open_challenge
