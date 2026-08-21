#pragma once

#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

#include <sfTk/sfTkII2C.h>

class LinuxI2C : public sfTkII2C {
  public:
	LinuxI2C();
	~LinuxI2C();

	bool openBus(std::uint8_t bus_num);
	void closeBus();

	sfTkError_t ping() override;

	sfTkError_t writeRegister(uint8_t *devReg, size_t regLength,
		const uint8_t *data, size_t length) override;

	sfTkError_t readRegister(uint8_t *devReg, size_t regLength, uint8_t *data,
		size_t length) override;

  private:
	int fd_ = -1;
};