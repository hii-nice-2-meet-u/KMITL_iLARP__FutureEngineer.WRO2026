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

inline navigation::NavigationConfig make_navigation_config() {
	navigation::NavigationConfig config;
	config.target_outer_distance_m = 0.18f;
	config.search_speed_mps = 0.234f;
	config.normal_speed_mps = 0.5f;
	config.approach_speed_mps = 0.45f;
	config.turning_speed_mps = 0.45f;

	config.max_acceleration_mps2 = 5.0f;
	config.max_deceleration_mps2 = 5.0f;

	config.approach_distance_m = 0.80f;
	config.turn_trigger_distance_m = 0.65f;
	config.turn_rearm_distance_m = 0.80f;
	config.turn_preview_time_s = 0.20f;
	config.corner_radius_m = 0.30f;

	config.turn_entry_blend_rad = 9.0f * PI / 180.0f;
	config.max_steering_rate_rad_s = 6.0f;

	config.stanley.k = 1.0f;
	config.stanley.max_steering_rad = 45.0f * PI / 180.0f;
	config.stanley.heading_pid.kp = 2.30f;
	config.stanley.heading_pid.ki = 0.12f;
	config.stanley.heading_pid.kd = 0.065f;

	config.turn_heading_pid.kp = 0.90f;
	config.turn_heading_pid.ki = 0.08f;
	config.turn_heading_pid.kd = 0.035f;

	config.speed_pid.kp = 4.0f;
	config.speed_pid.ki = 1.0f;
	config.speed_pid.kd = 0.04f;

	config.total_turns = 12;
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
			  << " accel=" << result.command.target_acceleration_mps2 << "m/s2"
			  << " steering=" << result.command.steering_rad * RAD_TO_DEG
			  << "deg wheel_rpm=" << wheel_rpm << " pulse=" << servo_pulse_us
			  << "us\n";
}

} // namespace open_challenge
