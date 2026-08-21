#include "otos.hpp"

namespace otos {

bool OTOS::initialize(const std::uint8_t &bus_num) {

	if (!bus_.openBus(bus_num)) {
		return false;
	}

	bus_.setAddress(kDefaultAddress);

	if (sfDevOTOS::begin(&bus_) != ksfTkErrOk) {
		return false;
	}

	setLinearUnit(kSfeOtosLinearUnitMeters);
	setAngularUnit(kSfeOtosAngularUnitRadians);

	return true;
}

void OTOS::delayMs(uint32_t ms) {
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

} // namespace otos