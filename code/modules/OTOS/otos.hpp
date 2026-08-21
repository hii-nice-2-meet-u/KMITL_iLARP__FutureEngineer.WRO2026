#pragma once

#include <sfTk/sfDevOTOS.h>
#include <chrono>
#include <thread>

#include "linux_i2c.hpp"

namespace otos {
class OTOS : public sfDevOTOS {
  public:
	OTOS() = default;
	~OTOS();

	bool initialize(const std::uint8_t &bus_num);

  protected:
	void delayMs(uint32_t ms) override;

  private:
	LinuxI2C bus_;
};

} // namespace otos
