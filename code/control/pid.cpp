#include "pid.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace control {

PID::PID(PIDConfig config) : config_(config) {}

float PID::calculate(float setpoint, float current) {
	const auto now = Clock::now();

	if (!initialized_) {
		previous_time_ = now;
		return calculate(setpoint, current, 0.0f);
	}

	const float dt = std::chrono::duration<float>(now - previous_time_).count();
	previous_time_ = now;
	return calculate(setpoint, current, dt);
}

float PID::calculate(float setpoint, float current, float dt_s) {

	const float error = setpoint - current;

	if (!initialized_) {

		previous_error_ = error;
		initialized_ = true;

		const float output = config_.kp * error;
		return std::clamp(output, config_.min_output, config_.max_output);
	}

	if (dt_s <= 0.0f) {

		const float output = config_.kp * error;
		previous_error_ = error;
		return std::clamp(output, config_.min_output, config_.max_output);
	}

	const float dt = std::clamp(dt_s, 1e-4f, std::max(1e-4f, config_.max_dt_s));

	const float p = config_.kp * error;

	const float candidate_integral = std::clamp(
		integral_ + error * dt, config_.min_integral, config_.max_integral);

	const float i = config_.ki * candidate_integral;
	const float derivative = (error - previous_error_) / dt;
	const float d = config_.kd * derivative;

	const float unsaturated_output = p + i + d;
	const float output =
		std::clamp(unsaturated_output, config_.min_output, config_.max_output);

	// Do not integrate farther into a saturated actuator command.
	if (output == unsaturated_output ||
		std::signbit(error) != std::signbit(unsaturated_output - output)) {
		integral_ = candidate_integral;
	}

	previous_error_ = error;

	return output;
}

void PID::reset() {

	integral_ = 0.0f;

	previous_error_ = 0.0f;

	previous_time_ = Clock::time_point{};

	initialized_ = false;
}

void PID::set_config(const PIDConfig &config) {

	config_ = config;

	reset();
}

} // namespace control
