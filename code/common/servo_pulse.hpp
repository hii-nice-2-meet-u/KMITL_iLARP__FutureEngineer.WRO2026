#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

// Pure servo-pulse arithmetic, extracted from ActuatorOutput so the exact
// steering-rad -> pulse mapping can be exercised off-robot without opening SPI.
//
// This header is the single source of truth for the conversion: ActuatorOutput
// delegates to it, and the navigation sim tests dump their pulse sequence
// through it. That shared path is what makes the "pulse-identical" verification
// bar (MOTION_CORE_AND_PATCH_PLAN.md PART 4) runnable.
//
// The functions are byte-for-byte the math that previously lived inside
// ActuatorOutput::to_servo_pulse_us / limit_servo_pulse_step. servo_pulse_test
// pins the numeric behaviour with golden values.
namespace actuator {

struct ServoPulseConfig {
	std::uint16_t servo_min_pulse_us{950};
	std::uint16_t servo_center_pulse_us{1475};
	std::uint16_t servo_max_pulse_us{2000};
	std::uint16_t maximum_servo_step_us{500};
	float steering_to_servo_sign{1.0f};
	float maximum_steering_command_deg{45.0f};
};

// Map a steering angle [rad] onto an absolute servo pulse [us]. Non-finite
// input maps to the centre pulse. The zero-steering pulse is the configured
// centre, and full command maps to min/max about that centre.
inline std::uint16_t to_servo_pulse_us(
	const ServoPulseConfig &config, float steering_rad) {
	const float maximum_steering_command_deg =
		std::max(0.1f, std::abs(config.maximum_steering_command_deg));
	const float steering_deg = std::clamp(std::isfinite(steering_rad)
			? steering_rad * 180.0f / 3.14159265358979323846f
			: 0.0f,
		-maximum_steering_command_deg, maximum_steering_command_deg);
	const float signed_command_deg =
		std::clamp(config.steering_to_servo_sign * steering_deg,
			-maximum_steering_command_deg, maximum_steering_command_deg);
	const float minimum_pulse_us = static_cast<float>(
		std::min(config.servo_min_pulse_us, config.servo_max_pulse_us));
	const float maximum_pulse_us = static_cast<float>(
		std::max(config.servo_min_pulse_us, config.servo_max_pulse_us));
	const float center_pulse_us = std::clamp(
		static_cast<float>(config.servo_center_pulse_us), minimum_pulse_us,
		maximum_pulse_us);
	const float normalized_command =
		signed_command_deg / maximum_steering_command_deg;
	const float pulse_span_us = normalized_command >= 0.0f
		? maximum_pulse_us - center_pulse_us
		: center_pulse_us - minimum_pulse_us;
	const float pulse_us = center_pulse_us + normalized_command * pulse_span_us;
	return static_cast<std::uint16_t>(
		std::lround(std::clamp(pulse_us, minimum_pulse_us, maximum_pulse_us)));
}

// Slew-limit the commanded pulse to at most maximum_servo_step_us away from the
// current pulse, so a single control step cannot demand an abrupt servo jump.
inline std::uint16_t limit_servo_pulse_step(const ServoPulseConfig &config,
	std::uint16_t target_pulse_us, std::uint16_t current_pulse_us) {
	const std::int32_t current = current_pulse_us;
	const std::int32_t maximum_step_us =
		std::max<std::int32_t>(1, config.maximum_servo_step_us);
	const std::int32_t pulse_error_us =
		static_cast<std::int32_t>(target_pulse_us) - current;
	return static_cast<std::uint16_t>(current +
		std::clamp(pulse_error_us, -maximum_step_us, maximum_step_us));
}

} // namespace actuator
