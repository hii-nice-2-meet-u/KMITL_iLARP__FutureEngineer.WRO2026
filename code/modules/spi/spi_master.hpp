#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include <linux/spi/spidev.h>

namespace spi {

enum class Command : std::uint8_t {
	NULL_ = 0x00,
	ECTO_TEST = 0x01,

	M_ENABLE = 0x20,  // M_CTRL
	M_DISABLE = 0x21, // M_CTRL
	M1_POW = 0x22,	  // M_CTRL
	M2_POW = 0x23,	  // M_CTRL
	M1_DUTY = 0x24,	  // M_CTRL
	M2_DUTY = 0x25,	  // M_CTRL

	M1_SPD = 0x26,		   // M_ENC
	M2_SPD = 0x27,		   // M_ENC
	M_Brake = 0x28,		   // M_ENC
	M_ENC_ENABLE = 0x29,   // M_ENC
	M_ENC_DISABLE = 0x30,  // M_ENC
	M_ENC_INVERTED = 0x31, // M_ENC

	SERVO_PULSE = 0x40,
	SERVO_ANGLE = 0x41,

	VOL_CHECK = 0xA0,

	DEBUG = 0xFF
};

enum class Motor { M1, M2 };

class SPI {
  public:
	SPI() = default;
	~SPI();

	SPI(const SPI &) = delete;
	SPI &operator=(const SPI &) = delete;

	bool initialize(std::uint8_t chip_select = 0,
		std::uint8_t mode = SPI_MODE_0, std::uint8_t bits = 8,
		std::uint32_t speed_hz = 15'000'000);

	void close();
	bool is_open() const { return fd_ >= 0; }

	bool echo_test(std::uint16_t value, std::uint16_t &response);
	bool enable_motors();
	bool disable_motors();

	// Signed motor power: -100 to +100. Negative values are encoded as signed
	// 16-bit two's complement in the frame data field.
	bool set_motor_power(Motor motor, std::int16_t percent);

	// Accepted range is 0-100. Values outside the range are rejected.
	bool set_motor_speed(Motor motor, std::uint16_t percent);

	// Positional servo pulse width: 1000-2100 microseconds.
	bool set_servo_pulse_us(std::uint16_t pulse_us);

	// Positional servo command: 0-180 degrees, center = 90 degrees.
	bool set_servo_angle(std::uint16_t angle_deg);

	// The controller returns battery voltage as unsigned millivolts.
	std::optional<float> read_voltage_v();

  private:
	using Frame = std::array<std::uint8_t, 3>;
	bool transfer(Command command, std::uint16_t data, Frame *rx = nullptr);

	bool request(Command command, std::uint16_t data, Frame &response);

	static std::uint16_t decode_data(const Frame &frame);

	int fd_{-1};
	std::uint32_t speed_hz_{0};
	std::uint8_t bits_{0};
};

} // namespace spi
