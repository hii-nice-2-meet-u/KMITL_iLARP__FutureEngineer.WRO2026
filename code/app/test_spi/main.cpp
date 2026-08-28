#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <thread>

#include "spi_master.hpp"

namespace {

constexpr std::int16_t TEST_POWER_PERCENT = 60;
constexpr auto TEST_DURATION = std::chrono::seconds(2);
constexpr auto STOP_CHECK_PERIOD = std::chrono::milliseconds(50);

std::atomic_bool stop_requested{false};

void request_stop(int) { stop_requested.store(true); }

class MotorStopGuard {
  public:
	explicit MotorStopGuard(spi::SPI &bus) : bus_(bus) {}

	~MotorStopGuard() { bus_.set_motor_power(spi::Motor::M1, 0); }

	MotorStopGuard(const MotorStopGuard &) = delete;
	MotorStopGuard &operator=(const MotorStopGuard &) = delete;

  private:
	spi::SPI &bus_;
};

} // namespace

int main() {
	std::signal(SIGINT, request_stop);
	std::signal(SIGTERM, request_stop);

	spi::SPI bus;
	if (!bus.initialize()) {
		std::cerr << "SPI initialization failed\n";
		return 1;
	}
	MotorStopGuard stop_guard(bus);

	if (!bus.set_motor_power(spi::Motor::M1, 0)) {
		std::cerr << "Cannot establish safe motor state\n";
		return 1;
	}

	std::cout << "M1_POW=60% TX=[0x22 0x00 0x3C]\n";
	if (!bus.set_motor_power(spi::Motor::M1, TEST_POWER_PERCENT)) {
		std::cerr << "Cannot set M1 power to 60%\n";
		return 1;
	}

	const auto stop_time = std::chrono::steady_clock::now() + TEST_DURATION;
	while (!stop_requested.load() &&
		std::chrono::steady_clock::now() < stop_time) {
		std::this_thread::sleep_for(STOP_CHECK_PERIOD);
	}

	std::cout << "M1_POW=0% TX=[0x22 0x00 0x00]\n";
	if (!bus.set_motor_power(spi::Motor::M1, 0)) {
		std::cerr << "Cannot set M1 power to 0%\n";
		return 1;
	}
	std::cout << "Test complete; M1 power is 0\n";
	return 0;
}
