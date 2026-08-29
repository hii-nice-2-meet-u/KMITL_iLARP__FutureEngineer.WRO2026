#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

#include "spi_master.hpp"

namespace {

void print_usage(const char *program) {
	std::cout << "Usage: " << program << " [options]\n\n"
			  << "Safe diagnostics (motors disabled by default):\n"
			  << "  --voltage                 Read slave battery voltage\n"
			  << "  --echo VALUE              Echo-test a 16-bit value\n"
			  << "  --servo-angle DEG        Set servo 0..180 degrees\n"
			  << "  --servo-pulse US          Set servo pulse 1000..2100 us\n\n"
			  << "Actuator tests (explicitly enable motors):\n"
			  << "  --motor-power PERCENT    M1 signed power -100..100\n"
			  << "  --motor-speed PERCENT    M1 speed command 0..100\n"
			  << "  --duration-ms MS          Hold command (default 500)\n"
			  << "  --help                    Show this help\n";
}

std::optional<long> parse_value(const char *text) {
	char *end = nullptr;
	const long value = std::strtol(text, &end, 0);
	if (end == text || *end != '\0') {
		return std::nullopt;
	}
	return value;
}

} // namespace

int main(int argc, char **argv) {
	spi::SPI bus;
	bool read_voltage = false;
	std::optional<std::uint16_t> echo_value;
	std::optional<std::uint16_t> servo_angle;
	std::optional<std::uint16_t> servo_pulse;
	std::optional<std::int16_t> motor_power;
	std::optional<std::uint16_t> motor_speed;
	int duration_ms = 500;

	for (int i = 1; i < argc; ++i) {
		const std::string option(argv[i]);
		if (option == "--help") {
			print_usage(argv[0]);
			return 0;
		}
		if (option == "--voltage") {
			read_voltage = true;
			continue;
		}
		if (i + 1 >= argc) {
			std::cerr << "Missing value for " << option << '\n';
			print_usage(argv[0]);
			return 2;
		}
		const auto value = parse_value(argv[++i]);
		if (!value.has_value()) {
			std::cerr << "Invalid numeric value for " << option << '\n';
			return 2;
		}
		if (option == "--echo" && *value >= 0 && *value <= 0xFFFF) {
			echo_value = static_cast<std::uint16_t>(*value);
		} else if (option == "--servo-angle" && *value >= 0 && *value <= 180) {
			servo_angle = static_cast<std::uint16_t>(*value);
		} else if (option == "--servo-pulse" && *value >= 1000 &&
			*value <= 2100) {
			servo_pulse = static_cast<std::uint16_t>(*value);
		} else if (option == "--motor-power" && *value >= -100 &&
			*value <= 100) {
			motor_power = static_cast<std::int16_t>(*value);
		} else if (option == "--motor-speed" && *value >= 0 && *value <= 100) {
			motor_speed = static_cast<std::uint16_t>(*value);
		} else if (option == "--duration-ms" && *value >= 0 &&
			*value <= 60000) {
			duration_ms = static_cast<int>(*value);
		} else {
			std::cerr << "Invalid or unknown option: " << option << '\n';
			print_usage(argv[0]);
			return 2;
		}
	}

	if (!bus.initialize()) {
		return 1;
	}

	bool motors_enabled = false;
	auto stop_motors = [&] {
		if (motors_enabled) {
			bus.set_motor_power(spi::Motor::M1, 0);
			bus.set_motor_power(spi::Motor::M2, 0);
			bus.disable_motors();
			motors_enabled = false;
		}
	};

	bool ok = true;
	if (read_voltage) {
		const auto voltage = bus.read_voltage_v();
		if (voltage.has_value()) {
			std::cout << "Voltage: " << *voltage << " V\n";
		} else {
			std::cerr << "Voltage read failed\n";
			ok = false;
		}
	}
	if (echo_value.has_value()) {
		std::uint16_t response = 0;
		if (bus.echo_test(*echo_value, response)) {
			std::cout << "Echo TX=0x" << std::hex << *echo_value << " RX=0x"
					  << response << std::dec << '\n';
		} else {
			std::cerr << "Echo test failed\n";
			ok = false;
		}
	}
	if (servo_angle.has_value()) {
		ok = bus.set_servo_angle(*servo_angle) && ok;
		std::cout << "Servo angle: " << *servo_angle << " deg\n";
	}
	if (servo_pulse.has_value()) {
		ok = bus.set_servo_pulse_us(*servo_pulse) && ok;
		std::cout << "Servo pulse: " << *servo_pulse << " us\n";
	}

	if (motor_power.has_value() || motor_speed.has_value()) {
		std::cout << "WARNING: enabling motors for explicit actuator test\n";
		if (!bus.enable_motors()) {
			std::cerr << "Motor enable failed\n";
			ok = false;
		} else {
			motors_enabled = true;
			if (motor_power.has_value()) {
				ok = bus.set_motor_power(spi::Motor::M1, *motor_power) && ok;
				std::cout << "M1 power: " << *motor_power << "%\n";
			}
			if (motor_speed.has_value()) {
				ok = bus.set_motor_speed(spi::Motor::M1, *motor_speed) && ok;
				std::cout << "M1 speed command: " << *motor_speed << "%\n";
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
		}
	}

	stop_motors();
	bus.close();
	return ok ? 0 : 1;
}
