#include "linux_i2c.hpp"

LinuxI2C::~LinuxI2C() { closeBus(); }

bool LinuxI2C::openBus(const std::uint8_t &bus_num) {

	fd_ = open(("/dev/i2c-" + std::to_string(bus_num)).c_str(), O_RDWR);

	if (fd_ < 0) {
		return false;
	}

	return true;
}

void LinuxI2C::closeBus() {
	if (fd_ >= 0) {
		close(fd_);
		fd_ = -1;
	}
}

sfTkError_t LinuxI2C::ping() {
	if (fd_ < 0) {
		return ksfTkErrBusNotInit;
	}

	if (ioctl(fd_, I2C_SLAVE, address()) < 0) {
		return ksfTkErrBusNoResponse;
	}

	return ksfTkErrOk;
}

sfTkError_t LinuxI2C::writeRegister(
	uint8_t *devReg, size_t regLength, const uint8_t *data, size_t length) {

	if (fd_ < 0) return ksfTkErrBusNotInit;

	if (ioctl(fd_, I2C_SLAVE, address()) < 0) return ksfTkErrBusNoResponse;

	std::vector<uint8_t> buffer;

	if (devReg != nullptr && regLength > 0) {
		buffer.insert(buffer.end(), devReg, devReg + regLength);
	}

	if (data != nullptr && length > 0) {
		buffer.insert(buffer.end(), data, data + length);
	}

	const ssize_t written = write(fd_, buffer.data(), buffer.size());

	if (written != static_cast<ssize_t>(buffer.size())) {
		return ksfTkErrFail;
	}

	return ksfTkErrOk;
}

sfTkError_t readRegister(
	uint8_t *devReg, size_t regLength, uint8_t *data, size_t length) {

	if (fd_ < 0) return ksfTkErrBusNotInit;

	if (ioctl(fd_, I2C_SLAVE, address()) < 0) return ksfTkErrBusNoResponse;

	if (devReg != nullptr && regLength > 0) {

		const ssize_t written = write(fd_, devReg, regLength);

		if (written != static_cast<ssize_t>(regLength)) {
			return ksfTkErrFail;
		}
	}

	const ssize_t received = read(fd_, data, length);

	if (received != static_cast<ssize_t>(length)) return ksfTkErrBusUnderRead;

	return ksfTkErrOk;
}