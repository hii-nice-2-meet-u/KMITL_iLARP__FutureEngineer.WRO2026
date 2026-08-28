#include "spi_master.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

namespace spi {

namespace {

constexpr std::uint8_t MAX_PERCENT = 100;
constexpr std::uint8_t MAX_SERVO_ANGLE_DEG = 180;
constexpr auto RESPONSE_DELAY = std::chrono::milliseconds(2);

} // namespace

SPI::~SPI() { close(); }

bool SPI::initialize(std::uint8_t chip_select, std::uint8_t mode,
	std::uint8_t bits, std::uint32_t speed_hz) {
	close();

	const std::string device = "/dev/spidev0." + std::to_string(chip_select);
	fd_ = open(device.c_str(), O_RDWR);
	if (fd_ < 0) {
		std::cerr << "Cannot open " << device << ": " << std::strerror(errno)
				  << '\n';
		return false;
	}

	if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
		ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
		ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) {
		std::cerr << "Cannot configure " << device << ": "
				  << std::strerror(errno) << '\n';
		close();
		return false;
	}

	speed_hz_ = speed_hz;
	bits_ = bits;
	return true;
}

void SPI::close() {
	if (fd_ >= 0) {
		::close(fd_);
		fd_ = -1;
	}
	speed_hz_ = 0;
	bits_ = 0;
}

bool SPI::echo_test(std::uint16_t value, std::uint16_t &response) {
	Frame frame{};
	if (!request(Command::ECTO_TEST, value, frame)) {
		return false;
	}
	response = decode_data(frame);
	return true;
}

bool SPI::enable_motors() { return transfer(Command::M_ENABLE, 0); }

bool SPI::disable_motors() { return transfer(Command::M_DISABLE, 0); }

bool SPI::set_motor_power(Motor motor, std::uint16_t percent) {
	if (percent > MAX_PERCENT) {
		std::cerr << "Motor power must be in range 0-100\n";
		return false;
	}
	const Command command =
		motor == Motor::M1 ? Command::M1_POW : Command::M2_POW;
	return transfer(command, percent);
}

bool SPI::set_motor_speed(Motor motor, std::uint16_t percent) {
	if (percent > MAX_PERCENT) {
		std::cerr << "Motor speed must be in range 0-100\n";
		return false;
	}
	const Command command =
		motor == Motor::M1 ? Command::M1_SPD : Command::M2_SPD;
	return transfer(command, percent);
}

bool SPI::set_servo_angle(std::uint16_t angle_deg) {
	if (angle_deg > MAX_SERVO_ANGLE_DEG) {
		std::cerr << "Servo angle must be in range 0-180 degrees\n";
		return false;
	}
	return transfer(Command::SERVO_ANGLE, angle_deg);
}

std::optional<float> SPI::read_voltage_v() {
	Frame frame{};
	if (!request(Command::VOL_CHECK, 0, frame)) {
		return std::nullopt;
	}
	return static_cast<float>(decode_data(frame)) / 1000.0f;
}

bool SPI::transfer(Command command, std::uint16_t data, Frame *rx) {
	if (!is_open()) {
		std::cerr << "SPI transfer requested while bus is closed\n";
		return false;
	}

	const Frame tx{static_cast<std::uint8_t>(command),
		static_cast<std::uint8_t>((data >> 8) & 0xFF),
		static_cast<std::uint8_t>(data & 0xFF)};
	Frame ignored_rx{};
	Frame &receive = rx != nullptr ? *rx : ignored_rx;

	spi_ioc_transfer transaction{};
	transaction.tx_buf = reinterpret_cast<std::uintptr_t>(tx.data());
	transaction.rx_buf = reinterpret_cast<std::uintptr_t>(receive.data());
	transaction.len = static_cast<std::uint32_t>(tx.size());
	transaction.speed_hz = speed_hz_;
	transaction.bits_per_word = bits_;

	if (ioctl(fd_, SPI_IOC_MESSAGE(1), &transaction) < 0) {
		std::cerr << "SPI transfer failed: " << std::strerror(errno) << '\n';
		return false;
	}
	return true;
}

bool SPI::request(Command command, std::uint16_t data, Frame &response) {
	if (!transfer(command, data)) {
		return false;
	}

	// The slave prepares a response after receiving a command. Clock it out in
	// a second frame using NULL_ without changing controller state.
	std::this_thread::sleep_for(RESPONSE_DELAY);
	return transfer(Command::NULL_, 0, &response);
}

std::uint16_t SPI::decode_data(const Frame &frame) {
	return static_cast<std::uint16_t>(
		(static_cast<std::uint16_t>(frame[1]) << 8) | frame[2]);
}

} // namespace spi
