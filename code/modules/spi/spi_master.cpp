#include "spi_master.hpp"
#include <cstdint>

namespace spi {

SPI::~SPI() { closeBus(); }

bool SPI::init(const std::uint8_t &num_port, std::uint8_t mode,
	std::uint8_t bits, std::uint32_t speed) {

	fd_ = open(("/dev/spidev0." + std::to_string(num_port)).c_str(), O_RDWR);

	if (fd_ < 0) {
		std::cerr << "Cannot open SPI port\n";
		return false;
	}

	if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
		ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
		ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {

		std::cerr << "Cannot configure SPI\n";
		close(fd_);

		return false;
	}

	speed_ = speed;
	bits_ = bits;

	return true;
}

void SPI::closeBus() {
	if (fd_ >= 0) {
		close(fd_);
		fd_ = -1;
	}
}

bool SPI::send_data(
	const Command &command, std::uint16_t tx_data, uint8_t *rx = nullptr) {

	spi_ioc_transfer tr{};
	std::uint8_t msb = static_cast<std::uint8_t>((tx_data >> 8) & 0xFF);
	std::uint8_t lsb = static_cast<std::uint8_t>(tx_data & 0xFF);

	const std::uint8_t cmd[3] = {static_cast<std::uint8_t>(command), msb, lsb};

	tr.tx_buf = reinterpret_cast<unsigned long>(cmd);

	tr.rx_buf = rx ? reinterpret_cast<uintptr_t>(rx) : 0;

	tr.len = static_cast<uint32_t>(sizeof(cmd));

	tr.speed_hz = speed;

	tr.bits_per_word = bits;

	return ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) >= 0;
}

std::optional<uint32_t> SPI::get_voltage() {
	std::uint8_t recieve[3]{};
	if (!send_data(Command::VOL_CHECK, 0x0000)) {
		return std::nullopt;;
	}

	if (!send_data(Command::NULL_, 0x0000, recieve)) {
		return std::nullopt;
	}

	int result = (static_cast<uint16_t>(receive[1]) << 8) | receive[2];
	return static_cast<float>(result / 1000.0f);
}

} // namespace spi