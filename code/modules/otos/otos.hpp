#pragma once

#include <chrono>
#include <thread>
#include <iostream>

#include "linux_i2c.hpp"
#include <sfTk/sfDevOTOS.h>

namespace otos {
class OTOS : public sfDevOTOS {
  public:
	OTOS() = default;
	~OTOS() = default;

	bool initialize(const std::uint8_t &bus_num = 1);

  protected:
	void delayMs(uint32_t ms) override;

  private:
	LinuxI2C bus_;
};

} // namespace otos
