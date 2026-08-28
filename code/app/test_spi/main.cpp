#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

#include "spi_master.hpp"

namespace {

void print_help() {
	std::cout << "\nCommands:\n"
			  << "  t VALUE       echo-test a 16-bit value\n"
			  << "  v             read voltage\n"
			  << "  e             enable motors\n"
			  << "  d             disable motors\n"
			  << "  p MOTOR VALUE set motor power, MOTOR=1|2, VALUE=0..100\n"
			  << "  s MOTOR VALUE set motor speed, MOTOR=1|2, VALUE=0..100\n"
			  << "  a ANGLE       set servo angle 0..180, center=90\n"
			  << "  h             show this help\n"
			  << "  q             stop motors and quit\n\n";
}

bool read_motor(int number, spi::Motor &motor) {
	if (number == 1) {
		motor = spi::Motor::M1;
		return true;
	}
	if (number == 2) {
		motor = spi::Motor::M2;
		return true;
	}
	std::cerr << "Motor must be 1 or 2\n";
	return false;
}

void clear_bad_input() {
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

bool stop_motors(spi::SPI &bus) {
	const bool m1_power = bus.set_motor_power(spi::Motor::M1, 0);
	const bool m2_power = bus.set_motor_power(spi::Motor::M2, 0);
	const bool m1_speed = bus.set_motor_speed(spi::Motor::M1, 0);
	const bool m2_speed = bus.set_motor_speed(spi::Motor::M2, 0);
	const bool disabled = bus.disable_motors();
	return m1_power && m2_power && m1_speed && m2_speed && disabled;
}

} // namespace

int main() {
	spi::SPI bus;
	if (!bus.initialize()) {
		return 1;
	}

	// Motor output starts disabled. Servo remains untouched until the user
	// explicitly enters an angle command.
	if (!bus.disable_motors()) {
		std::cerr << "Failed to put motor controller in disabled state\n";
		return 1;
	}

	std::cout << "SPI ready on /dev/spidev0.0 at 15 MHz\n"
			  << "Motors are DISABLED; servo has not been moved.\n";
	print_help();

	std::string command;
	while (std::cout << "spi> " && std::cin >> command) {
		if (command == "q") {
			break;
		}
		if (command == "h") {
			print_help();
			continue;
		}
		if (command == "e") {
			std::cout << (bus.enable_motors() ? "Motors ENABLED\n"
											  : "Enable failed\n");
			continue;
		}
		if (command == "d") {
			std::cout << (stop_motors(bus) ? "Motors stopped and DISABLED\n"
										   : "Motor stop failed\n");
			continue;
		}
		if (command == "v") {
			const auto voltage = bus.read_voltage_v();
			if (voltage.has_value()) {
				std::cout << "Voltage: " << *voltage << " V\n";
			} else {
				std::cout << "Voltage read failed\n";
			}
			continue;
		}
		if (command == "t") {
			unsigned int value = 0;
			if (!(std::cin >> value) || value > 65535) {
				std::cerr << "Echo value must be 0-65535\n";
				clear_bad_input();
				continue;
			}
			std::uint16_t response = 0;
			if (bus.echo_test(static_cast<std::uint16_t>(value), response)) {
				std::cout << "Echo response: " << response
						  << (response == value ? " PASS\n" : " MISMATCH\n");
			} else {
				std::cout << "Echo transfer failed\n";
			}
			continue;
		}
		if (command == "a") {
			unsigned int angle = 0;
			if (!(std::cin >> angle) || angle > 180) {
				std::cerr << "Servo angle must be 0-180\n";
				clear_bad_input();
				continue;
			}
			std::cout << (bus.set_servo_angle(angle)
					? "Servo angle sent\n"
					: "Servo command failed\n");
			continue;
		}
		if (command == "p" || command == "s") {
			int motor_number = 0;
			unsigned int value = 0;
			if (!(std::cin >> motor_number >> value) || value > 100) {
				std::cerr << "Expected MOTOR=1|2 and VALUE=0..100\n";
				clear_bad_input();
				continue;
			}
			spi::Motor motor = spi::Motor::M1;
			if (!read_motor(motor_number, motor)) {
				continue;
			}
			const bool ok = command == "p" ? bus.set_motor_power(motor, value)
										   : bus.set_motor_speed(motor, value);
			std::cout << (ok ? "Motor command sent\n"
							 : "Motor command failed\n");
			continue;
		}

		std::cerr << "Unknown command; enter h for help\n";
	}

	if (!stop_motors(bus)) {
		std::cerr << "WARNING: final motor-disable command failed\n";
	}
	bus.close();
	std::cout << "SPI closed\n";
	return 0;
}
