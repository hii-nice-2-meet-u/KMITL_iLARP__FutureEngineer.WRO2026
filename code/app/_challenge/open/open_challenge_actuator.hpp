#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "navigation_state.hpp"
#include "spi_master.hpp"

namespace open_challenge {

struct ActuatorConfig {
	std::uint8_t spi_chip_select{0};
	std::uint32_t spi_speed_hz{15'000'000};

	float full_scale_speed_mps{0.85f};
	std::uint16_t minimum_moving_power{35};
	std::uint16_t maximum_drive_percent{100};

	std::uint16_t servo_min_pulse_us{1000};
	std::uint16_t servo_max_pulse_us{2100};
	float steering_to_servo_sign{1.0f};
	float maximum_wheel_angle_deg{45.0f};
};

struct ActuatorTelemetry {
	std::int16_t power_percent{0};
	std::uint16_t servo_pulse_us{1550};
	bool armed{false};
};

class ActuatorOutput {
  public:
	explicit ActuatorOutput(ActuatorConfig config = {}) : config_(config) {}

	~ActuatorOutput() {
		if (initialized_) {
			emergency_stop();
		}
	}

	ActuatorOutput(const ActuatorOutput &) = delete;
	ActuatorOutput &operator=(const ActuatorOutput &) = delete;

	bool initialize() {
		if (!bus_.initialize(
				config_.spi_chip_select, SPI_MODE_0, 8, config_.spi_speed_hz)) {
			return false;
		}
		initialized_ = true;

		if (!write_motor_power(0) ||
			!bus_.set_servo_pulse_us(to_servo_pulse_us(0.0f))) {
			emergency_stop();
			return false;
		}

		telemetry_ = {};
		telemetry_.servo_pulse_us = to_servo_pulse_us(0.0f);
		return true;
	}

	bool arm() {
		if (!initialized_) {
			return false;
		}
		if (!write_motor_power(0)) {
			emergency_stop();
			return false;
		}
		telemetry_.armed = true;
		return true;
	}

	bool apply(const navigation::NavigationCommand &command) {
		if (!initialized_ || !telemetry_.armed) {
			return false;
		}

		const std::uint16_t servo_pulse_us =
			to_servo_pulse_us(command.steering_rad);
		const std::int16_t power_percent =
			to_power_percent(command.target_speed_mps);

		if (!bus_.set_servo_pulse_us(servo_pulse_us) ||
			!write_motor_power(power_percent)) {
			emergency_stop();
			return false;
		}

		telemetry_.servo_pulse_us = servo_pulse_us;
		telemetry_.power_percent = power_percent;
		return true;
	}

	bool emergency_stop() {
		if (!initialized_) {
			return true;
		}

		const bool drive_zero = write_motor_power(0);
		telemetry_.power_percent = 0;
		telemetry_.armed = false;
		return drive_zero;
	}

	void close() {
		if (initialized_) {
			emergency_stop();
			bus_.close();
			initialized_ = false;
		}
	}

	const ActuatorTelemetry &telemetry() const { return telemetry_; }
	const ActuatorConfig &config() const { return config_; }

  private:
	std::int16_t to_power_percent(float target_speed_mps) const {
		const float full_scale_mps =
			std::max(0.01f, config_.full_scale_speed_mps);
		const float normalized = std::clamp(std::isfinite(target_speed_mps)
				? target_speed_mps / full_scale_mps
				: 0.0f,
			0.0f, 1.0f);
		const float limited_percent = normalized *
			static_cast<float>(
				std::min<std::uint16_t>(config_.maximum_drive_percent, 100));
		return static_cast<std::int16_t>(std::lround(limited_percent));
	}

	std::uint16_t to_servo_pulse_us(float steering_rad) const {
		const float maximum_wheel_angle_deg =
			std::max(0.1f, std::abs(config_.maximum_wheel_angle_deg));
		const float steering_deg = std::clamp(std::isfinite(steering_rad)
				? steering_rad * 180.0f / 3.14159265358979323846f
				: 0.0f,
			-maximum_wheel_angle_deg, maximum_wheel_angle_deg);
		const float pulse_angle_deg =
			std::clamp(config_.steering_to_servo_sign * steering_deg,
				-maximum_wheel_angle_deg, maximum_wheel_angle_deg);
		const float normalized = (pulse_angle_deg + maximum_wheel_angle_deg) /
			(2.0f * maximum_wheel_angle_deg);
		const float minimum_pulse_us = static_cast<float>(
			std::min(config_.servo_min_pulse_us, config_.servo_max_pulse_us));
		const float maximum_pulse_us = static_cast<float>(
			std::max(config_.servo_min_pulse_us, config_.servo_max_pulse_us));
		const float pulse_us = minimum_pulse_us +
			normalized * (maximum_pulse_us - minimum_pulse_us);
		return static_cast<std::uint16_t>(
			std::lround(std::clamp(pulse_us, 1000.0f, 2100.0f)));
	}

	bool write_motor_power(std::int16_t percent) {
		return bus_.set_motor_power(spi::Motor::M1, percent);
	}

	ActuatorConfig config_;
	spi::SPI bus_;
	ActuatorTelemetry telemetry_;
	bool initialized_{false};
};

}
