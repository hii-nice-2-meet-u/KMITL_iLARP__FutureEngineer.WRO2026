#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

#define SPI_DEVICE "/dev/spidev0.0"

bool spi_transfer(int fd, const uint8_t *tx, uint8_t *rx, std::size_t len,
	uint32_t speed, uint8_t bits) {

	spi_ioc_transfer tr{};

	tr.tx_buf = reinterpret_cast<unsigned long>(tx);

	tr.rx_buf = reinterpret_cast<unsigned long>(rx);

	tr.len = static_cast<uint32_t>(len);

	tr.speed_hz = speed;

	tr.bits_per_word = bits;

	return ioctl(fd, SPI_IOC_MESSAGE(1), &tr) >= 0;
}

void print_rx(const uint8_t *rx, std::size_t len) {

	std::cout << "RX: ";

	for (std::size_t i = 0; i < len; ++i) {

		std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0')
				  << static_cast<int>(rx[i]) << ' ';
	}

	std::cout << std::dec << '\n';
}

int main() {

	const int fd = open(SPI_DEVICE, O_RDWR);

	if (fd < 0) {

		std::cerr << "Cannot open SPI port\n";

		return 1;
	}

	uint8_t mode = SPI_MODE_0;

	uint8_t bits = 8;

	uint32_t speed = 25'000'000;

	if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
		ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
		ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {

		std::cerr << "Cannot configure SPI\n";

		close(fd);

		return 1;
	}

	const uint8_t command[]{0xFF, 0x00, 0x4B};

	const uint8_t dummy[]{0x00, 0x00, 0x00};

	while (true) {

		// -----------------------------------------------------
		// 1. Send FF 00 4B
		// -----------------------------------------------------

		uint8_t rx_command[3]{};

		if (!spi_transfer(
				fd, command, rx_command, sizeof(command), speed, bits)) {

			std::cerr << "SPI command transfer failed\n";

			break;
		}

		std::cout << "TX: FF 00 4B  ";

		print_rx(rx_command, sizeof(rx_command));

		// -----------------------------------------------------
		// 2. Send 00 00 00
		// -----------------------------------------------------
		std::this_thread::sleep_for(std::chrono::seconds(1));

		uint8_t rx_data[3]{};

		if (!spi_transfer(fd, dummy, rx_data, sizeof(dummy), speed, bits)) {

			std::cerr << "SPI data transfer failed\n";

			break;
		}

		std::cout << "TX: 00 00 00  ";

		print_rx(rx_data, sizeof(rx_data));

		// -----------------------------------------------------
		// 3. Wait 1 second
		// -----------------------------------------------------

		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	close(fd);

	return 0;
}