#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <optional>

#include "navigation_state.hpp"
#include "spi_master.hpp"

namespace open_challenge {

struct ActuatorConfig {
	std::uint8_t spi_chip_select{0};
	std::uint32_t spi_speed_hz{15'000'000};

	float wheel_diameter_m{0.053f};
	std::uint16_t maximum_wheel_rpm{1500};

	std::uint16_t servo_min_pulse_us{950};
	std::uint16_t servo_center_pulse_us{1475};
	std::uint16_t servo_max_pulse_us{2000};
	std::uint16_t maximum_servo_step_us{500};
	float steering_to_servo_sign{1.0f};
	float maximum_steering_command_deg{45.0f};
};

struct ActuatorTelemetry {
	std::int16_t wheel_rpm{0};
	std::uint16_t servo_pulse_us{1475};
	bool armed{false};
};

class ActuatorOutput {
  public:
	explicit ActuatorOutput(ActuatorConfig config = {}) : config_(config) {}

	~ActuatorOutput() {
		if (initialized_) {
			close();
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

		if (!write_motor_rpm(0) ||
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
		if (!write_motor_rpm(0)) {
			emergency_stop();
			return false;
		}
		telemetry_.armed = true;
		return true;
	}

	bool apply(const navigation::NavigationCommand &command,
		std::optional<std::int16_t> wheel_rpm_override = std::nullopt) {
		if (!initialized_ || !telemetry_.armed) {
			return false;
		}

		const std::uint16_t target_servo_pulse_us =
			to_servo_pulse_us(command.steering_rad);
		const std::uint16_t servo_pulse_us =
			limit_servo_pulse_step(target_servo_pulse_us);
		const std::int16_t wheel_rpm = wheel_rpm_override.has_value()
			? clamp_wheel_rpm(*wheel_rpm_override)
			: to_wheel_rpm(command.target_speed_mps);

		if (!bus_.set_servo_pulse_us(servo_pulse_us)) {
			emergency_stop();
			return false;
		}
		if (wheel_rpm != telemetry_.wheel_rpm && !write_motor_rpm(wheel_rpm)) {
			emergency_stop();
			return false;
		}

		telemetry_.servo_pulse_us = servo_pulse_us;
		telemetry_.wheel_rpm = wheel_rpm;
		return true;
	}

	bool emergency_stop() {
		if (!initialized_) {
			return true;
		}

		const bool m1_zero = bus_.set_motor_speed(spi::Motor::M1, 0);
		const bool m2_zero = bus_.set_motor_speed(spi::Motor::M2, 0);
		const bool brake_ok = bus_.brake();
		const std::uint16_t center_pulse_us = to_servo_pulse_us(0.0f);
		const bool servo_centered = bus_.set_servo_pulse_us(center_pulse_us);
		telemetry_.wheel_rpm = 0;
		if (servo_centered) {
			telemetry_.servo_pulse_us = center_pulse_us;
		}
		telemetry_.armed = false;
		return m1_zero && m2_zero && brake_ok && servo_centered;
	}

	bool close() {
		if (initialized_) {
			const bool stopped = emergency_stop();
			bus_.close();
			initialized_ = false;
			return stopped;
		}
		return true;
	}

	std::optional<float> get_voltage() {
		if (!initialized_) {
			return std::nullopt;
		}
		return bus_.read_voltage_v();
	}

	const ActuatorTelemetry &telemetry() const { return telemetry_; }
	const ActuatorConfig &config() const { return config_; }

  private:
	std::int16_t to_wheel_rpm(float target_speed_mps) const {
		const float diameter_m = std::max(0.001f, config_.wheel_diameter_m);
		const float speed_mps =
			std::isfinite(target_speed_mps) ? target_speed_mps : 0.0f;
		const float rpm =
			speed_mps * 60.0f / (3.14159265358979323846f * diameter_m);
		const float maximum_rpm = static_cast<float>(
			std::clamp<std::uint16_t>(config_.maximum_wheel_rpm, 1,
				std::numeric_limits<std::int16_t>::max()));
		return static_cast<std::int16_t>(
			std::lround(std::clamp(rpm, -maximum_rpm, maximum_rpm)));
	}

	std::int16_t clamp_wheel_rpm(std::int16_t rpm) const {
		const std::int32_t maximum_rpm = std::clamp<std::int32_t>(
			config_.maximum_wheel_rpm, 1,
			std::numeric_limits<std::int16_t>::max());
		return static_cast<std::int16_t>(
			std::clamp<std::int32_t>(rpm, -maximum_rpm, maximum_rpm));
	}

	std::uint16_t to_servo_pulse_us(float steering_rad) const {
		const float maximum_steering_command_deg =
			std::max(0.1f, std::abs(config_.maximum_steering_command_deg));
		const float steering_deg = std::clamp(std::isfinite(steering_rad)
				? steering_rad * 180.0f / 3.14159265358979323846f
				: 0.0f,
			-maximum_steering_command_deg, maximum_steering_command_deg);
		const float signed_command_deg =
			std::clamp(config_.steering_to_servo_sign * steering_deg,
				-maximum_steering_command_deg, maximum_steering_command_deg);
		const float minimum_pulse_us = static_cast<float>(
			std::min(config_.servo_min_pulse_us, config_.servo_max_pulse_us));
		const float maximum_pulse_us = static_cast<float>(
			std::max(config_.servo_min_pulse_us, config_.servo_max_pulse_us));
		const float center_pulse_us = std::clamp(
			static_cast<float>(config_.servo_center_pulse_us), minimum_pulse_us,
			maximum_pulse_us);
		const float normalized_command =
			signed_command_deg / maximum_steering_command_deg;
		const float pulse_span_us = normalized_command >= 0.0f
			? maximum_pulse_us - center_pulse_us
			: center_pulse_us - minimum_pulse_us;
		const float pulse_us =
			center_pulse_us + normalized_command * pulse_span_us;
		return static_cast<std::uint16_t>(
			std::lround(std::clamp(pulse_us, minimum_pulse_us, maximum_pulse_us)));
	}

	std::uint16_t limit_servo_pulse_step(std::uint16_t target_pulse_us) const {
		const std::int32_t current_pulse_us = telemetry_.servo_pulse_us;
		const std::int32_t maximum_step_us =
			std::max<std::int32_t>(1, config_.maximum_servo_step_us);
		const std::int32_t pulse_error_us =
			static_cast<std::int32_t>(target_pulse_us) - current_pulse_us;
		return static_cast<std::uint16_t>(current_pulse_us +
			std::clamp(pulse_error_us, -maximum_step_us, maximum_step_us));
	}

	bool write_motor_rpm(std::int16_t rpm) {
		const bool m1_ok = bus_.set_motor_speed(spi::Motor::M1, rpm);
		const bool m2_ok = bus_.set_motor_speed(spi::Motor::M2, rpm);
		return m1_ok && m2_ok;
	}

	ActuatorConfig config_;
	spi::SPI bus_;
	ActuatorTelemetry telemetry_;
	bool initialized_{false};
};

}
