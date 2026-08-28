#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <optional>
#include <ostream>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

namespace spi {

enum Command : uint8_t {
	NULL_ = 0x00,
	ECTO_TEST = 0x01,

	M_ENABLE = 0x20,
	M_DISABLE = 0x21,
	M1_POW = 0x20, //LEFT
	M2_POW = 0x21, //RIGHT
	M1_SPD = 0x26,
	M2_SPD = 0x27,
	
	SERVO_PULSE = 0x40,
	SERVO_ANGLE = 0x41,
	
	VOL_CHECK = 0xa0;
};

class SPI {
  public:
	SPI();
	~SPI();

	bool init(const std::uint8_t &num_port = 0, std::uint8_t mode = SPI_MODE_0,
		std::uint8_t bits = 8, std::uint32_t speed = 15'000'000) const;
	void closeBus();

	std::optional<uint32_t> get_voltage();

  private:
	bool send_data(const Command &command, std::uint16_t tx_data, uint8_t &rx);

  private:
	int fd_{-1};
	std::uint32_t speed_{0};
	std::uint8_t bits_{0};
};
} // namespace spi