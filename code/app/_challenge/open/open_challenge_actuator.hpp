#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <optional>

#include "navigation_state.hpp"
#include "run_metadata.hpp"
#include "servo_pulse.hpp"
#include "spi_master.hpp"

namespace open_challenge {

struct ActuatorConfig {
	std::uint8_t spi_chip_select{0};
	std::uint32_t spi_speed_hz{15'000'000};

	float wheel_diameter_m{0.053f};
	std::uint16_t maximum_wheel_rpm{1500};

	// Pi-side compensation for the STM32 RPM scale. A commanded RPM produces
	// ~1.75x the intended ground speed (wheel measured 53.5 mm, so not the
	// wheel; OTOS off only -8.7%, so not the sensor -- see
	// docs/audit/HARDWARE_CHECKS.md and VEHICLE_MECHANICS_REVIEW).
	//
	// PROVISIONAL: 0.571 = 1/1.75 is a SINGLE-POINT correction fitted to one
	// operating point, NOT an identified gain. The command->speed relation is
	// not a pure scalar -- the NORMAL(~1.86)/TURNING(~1.51) spread shows a
	// steering-angle-dependent term a scalar cannot remove. Replace with a
	// measured model once M-3c/M-7 land (see MOTION_MODEL_UNIFIED_PLAN.md).
	// 1.0 = no compensation.
	float motor_rpm_command_scale{1.0f};

	std::uint16_t servo_min_pulse_us{950};
	std::uint16_t servo_center_pulse_us{1475};
	std::uint16_t servo_max_pulse_us{2000};
	std::uint16_t maximum_servo_step_us{500};
	float steering_to_servo_sign{1.0f};
	float maximum_steering_command_deg{45.0f};
};

struct ActuatorTelemetry {
	std::int16_t wheel_rpm{0};
	// servo_pulse_us is the value actually sent, after limit_servo_pulse_step.
	// commanded_servo_pulse_us is the value requested before that per-tick step
	// clamp, so a reader can see when maximum_servo_step_us is clipping the
	// steering command rather than inferring it from consecutive deltas.
	std::uint16_t servo_pulse_us{1475};
	std::uint16_t commanded_servo_pulse_us{1475};
	bool armed{false};
};

inline logging::JsonObject actuator_config_json(const ActuatorConfig &config) {
	logging::JsonObject object;
	object.add_unsigned("spi_chip_select", config.spi_chip_select)
		.add_unsigned("spi_speed_hz", config.spi_speed_hz)
		.add_number("wheel_diameter_m", config.wheel_diameter_m)
		.add_unsigned("maximum_wheel_rpm", config.maximum_wheel_rpm)
		.add_number(
			"motor_rpm_command_scale", config.motor_rpm_command_scale)
		.add_unsigned("servo_min_pulse_us", config.servo_min_pulse_us)
		.add_unsigned("servo_center_pulse_us", config.servo_center_pulse_us)
		.add_unsigned("servo_max_pulse_us", config.servo_max_pulse_us)
		.add_unsigned("maximum_servo_step_us", config.maximum_servo_step_us)
		.add_number(
			"steering_to_servo_sign", config.steering_to_servo_sign)
		.add_number("maximum_steering_command_deg",
			config.maximum_steering_command_deg);
	return object;
}

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
		telemetry_.commanded_servo_pulse_us = telemetry_.servo_pulse_us;
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
		telemetry_.commanded_servo_pulse_us = target_servo_pulse_us;
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
			telemetry_.commanded_servo_pulse_us = center_pulse_us;
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
		// motor_rpm_command_scale corrects the STM32 RPM-scale error (see
		// ActuatorConfig). It is applied only here, on the speed->RPM path, not
		// to wheel_rpm_override: the search-launch boost is an empirical max
		// kick, not a speed target, and its release is speed-gated by OTOS, so
		// scaling it would only weaken the launch.
		const float rpm = speed_mps * 60.0f /
			(3.14159265358979323846f * diameter_m) *
			config_.motor_rpm_command_scale;
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

	// Both conversions delegate to the shared, test-pinned arithmetic in
	// common/servo_pulse.hpp so the sim tests exercise the exact same mapping.
	actuator::ServoPulseConfig servo_pulse_config() const {
		return {config_.servo_min_pulse_us, config_.servo_center_pulse_us,
			config_.servo_max_pulse_us, config_.maximum_servo_step_us,
			config_.steering_to_servo_sign,
			config_.maximum_steering_command_deg};
	}

	std::uint16_t to_servo_pulse_us(float steering_rad) const {
		return actuator::to_servo_pulse_us(servo_pulse_config(), steering_rad);
	}

	std::uint16_t limit_servo_pulse_step(std::uint16_t target_pulse_us) const {
		return actuator::limit_servo_pulse_step(
			servo_pulse_config(), target_pulse_us, telemetry_.servo_pulse_us);
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
