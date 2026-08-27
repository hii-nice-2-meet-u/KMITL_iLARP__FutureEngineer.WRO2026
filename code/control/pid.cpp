#include "pid.hpp"

#include <algorithm>
#include <chrono>

namespace control {

PID::PID(PIDConfig config) : config_(config) {}

float PID::calculate(float setpoint, float current) {

	const float error = setpoint - current;

	const auto now = Clock::now();

	if (!initialized_) {

		previous_error_ = error;
		previous_time_ = now;
		initialized_ = true;

		const float output = config_.kp * error;
		return std::clamp(output, config_.min_output, config_.max_output);
	}

	float dt = std::chrono::duration<float>(now - previous_time_).count();
	previous_time_ = now;

	if (dt <= 0.0f) {

		const float output = config_.kp * error;
		previous_error_ = error;
		return std::clamp(output, config_.min_output, config_.max_output);
	}

	dt = std::min(dt, config_.max_dt_s);

	const float p = config_.kp * error;

	integral_ += error * dt;
	integral_ =
		std::clamp(integral_, config_.min_integral, config_.max_integral);
	
	const float i = config_.ki * integral_;
	const float derivative = (error - previous_error_) / dt;
	const float d = config_.kd * derivative;

	previous_error_ = error;

	const float output = p + i + d;

	return std::clamp(output, config_.min_output, config_.max_output);
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