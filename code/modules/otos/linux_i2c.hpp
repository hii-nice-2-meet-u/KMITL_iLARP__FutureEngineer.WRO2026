#pragma once

#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>
#include <thread>
#include <chrono>

#include <sfTk/sfTkII2C.h>

class LinuxI2C : public sfTkII2C {
  public:
	LinuxI2C() = default;
	~LinuxI2C();

	bool openBus(const std::uint8_t &bus_num);
	void closeBus();

	sfTkError_t ping() override;

	sfTkError_t writeRegister(
		uint8_t *devReg, size_t regLength, const uint8_t *data, size_t length);

	sfTkError_t readRegister(uint8_t *devReg, size_t regLength,
		uint8_t *data, size_t numBytes, size_t &readBytes,
		uint32_t read_delay) override;

  private:
	int fd_ = -1;
};