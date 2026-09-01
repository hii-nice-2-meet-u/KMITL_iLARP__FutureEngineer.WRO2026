#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

#include "spi_master.hpp"

namespace {

void print_usage(const char *program) {
	std::cout
		<< "Usage: " << program << " [options]\n\n"
		<< "  " << program
		<< " RPM                  M1=+RPM, M2=-RPM (0..1500)\n"
		<< "Diagnostics and raw transfer:\n"
		<< "  --voltage                 Read slave battery voltage\n"
		<< "  --echo VALUE              Echo-test a 16-bit value\n"
		<< "  --servo-angle DEG        Set servo 0..180 degrees\n"
		<< "  --raw B0 B1 B2          Raw full-duplex 3-byte transfer\n"
		<< "  --servo-pulse US          Send raw uint16 servo pulse\n\n"
		<< "Direct motor tests without enable/disable commands:\n"
		<< "  --motor-speed RPM        M1=+RPM, M2=-RPM (0..1500)\n"
		<< "  --duration-ms MS          Motor hold / raw delay (default 500)\n"
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

}

int main(int argc, char **argv) {
	spi::SPI bus;
	bool read_voltage = false;
	std::optional<std::uint16_t> echo_value;
	std::optional<std::uint16_t> servo_angle;
	std::optional<std::uint16_t> servo_pulse;
	std::optional<std::int16_t> motor_speed;
	std::optional<spi::SPI::Frame> raw_tx;
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
		if (option == "--raw") {
			if (i + 3 >= argc) {
				std::cerr << "--raw requires exactly 3 byte values\n";
				return 2;
			}
			spi::SPI::Frame frame{};
			for (std::size_t byte_index = 0; byte_index < frame.size();
				 ++byte_index) {
				const auto byte = parse_value(argv[++i]);
				if (!byte.has_value() || *byte < 0 || *byte > 0xFF) {
					std::cerr << "Raw bytes must be in range 0..255\n";
					return 2;
				}
				frame[byte_index] = static_cast<std::uint8_t>(*byte);
			}
			raw_tx = frame;
			continue;
		}
		if (!option.empty() && option.front() != '-') {
			const auto value = parse_value(option.c_str());
			if (!value.has_value() || *value < 0 || *value > 1500 ||
				motor_speed.has_value()) {
				std::cerr
					<< "Motor speed must be one value in range 0..1500 RPM\n";
				return 2;
			}
			motor_speed = static_cast<std::int16_t>(*value);
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
		} else if (option == "--servo-pulse" && *value >= 0 &&
			*value <= 0xFFFF) {
			servo_pulse = static_cast<std::uint16_t>(*value);
		} else if (option == "--motor-speed" && *value >= 0 && *value <= 1500) {
			motor_speed = static_cast<std::int16_t>(*value);
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

	bool ok = true;
	bool motors_enabled = false;
	auto stop_motors = [&] {
		if (motors_enabled) {
			const bool m1_stopped = bus.set_motor_speed(spi::Motor::M1, 0);
			const bool m2_stopped = bus.set_motor_speed(spi::Motor::M2, 0);
			const bool brake_ok = bus.brake();
			ok = m1_stopped && m2_stopped && brake_ok && ok;
			std::cout << "Motor stop: M1=0 RPM M2=0 RPM brake=ON\n";
			motors_enabled = false;
		}
	};

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
	if (raw_tx.has_value()) {
		spi::SPI::Frame raw_rx{};
		if (bus.transfer(*raw_tx, raw_rx)) {
			std::cout << "Raw TX:";
			for (const auto byte : *raw_tx) {
				std::cout << " 0x" << std::hex << std::setw(2)
						  << std::setfill('0') << static_cast<int>(byte);
			}
			std::cout << "  RX:";
			for (const auto byte : raw_rx) {
				std::cout << " 0x" << std::hex << std::setw(2)
						  << std::setfill('0') << static_cast<int>(byte);
			}
			std::cout << std::dec << '\n';
			std::cout << "Raw delay: " << duration_ms << " ms\n";
			std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
		} else {
			std::cerr << "Raw SPI transfer failed\n";
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

	if (motor_speed.has_value()) {
		motors_enabled = true;
		const std::int16_t m1_rpm = *motor_speed;
		const std::int16_t m2_rpm = *motor_speed;
		const bool m1_ok = bus.set_motor_speed(spi::Motor::M1, m1_rpm);
		const bool m2_ok = bus.set_motor_speed(spi::Motor::M2, m2_rpm);
		ok = m1_ok && m2_ok && ok;
		std::cout << "Motor speed: M1=" << m1_rpm << " RPM M2=" << m2_rpm
				  << " RPM\n";
		if (ok) {
			std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
		}
	}

	stop_motors();
	bus.close();
	return ok ? 0 : 1;
}
