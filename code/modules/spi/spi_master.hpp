#include <fcntl.h>
#include <iostream>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define SPI_DEVICE "/dev/spidev0.0"

int main() {
	int fd = open(SPI_DEVICE, O_RDWR);
	if (fd < 0) {
		std::cerr << "Cannot open SPI port" << std::endl;
		return 1;
	}

	uint8_t mode = SPI_MODE_0;
	uint8_t bits = 8;
	uint32_t speed = 1000000; // ความเร็ว 1 MHz

	ioctl(fd, SPI_IOC_WR_MODE, &mode);
	ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

	uint8_t tx[] = {0x9F, 0xFF, 0xFF}; // ตัวอย่างส่งคำสั่ง Read ID
	uint8_t rx[3] = {0};

	struct spi_ioc_transfer tr = {};
	tr.tx_buf = (unsigned long)tx;
	tr.rx_buf = (unsigned long)rx;
	tr.len = sizeof(tx);
	tr.speed_hz = speed;
	tr.bits_per_word = bits;

	if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
		std::cerr << "Cannot send data SPI (SPI_IOC_MESSAGE) ได้" << std::endl;
	} else {
		std::cout << "Succesfully recieved data: " << std::hex << (int)rx[0]
				  << " " << (int)rx[1] << " " << (int)rx[2] << std::endl;
	}

	uint8_t tx[] = {0x00, 0x00, 0x00};
	if(ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
		std::cerr << "Cannot send data SPI (SPI_IOC_MESSAGE) ได้" << std::endl;
	}
	std::this_thread::sleep_for(std::chrono::seconds(1));
	uint8_t tx[] = {0xFF, 0x00, 0x00};
	if(ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
		std::cerr << "Cannot send data SPI (SPI_IOC_MESSAGE) ได้" << std::endl;
	}
	close(fd);
	return 0;
}
