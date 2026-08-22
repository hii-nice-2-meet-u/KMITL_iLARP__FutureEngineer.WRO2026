#include <chrono>
#include <iostream>
#include <thread>

#include "otos.hpp"

int main() {
	otos::OTOS otos;

	std::cout << "Initializing OTOS...\n";

	if (!otos.initialize(1)) {
		std::cerr << "OTOS initialize failed\n";
		return 1;
	}

	std::cout << "OTOS connected\n";

	sfe_otos_version_t hw;
	sfe_otos_version_t fw;

	if (otos.getVersionInfo(hw, fw) == ksfTkErrOk) {
		std::cout << "HW: " << static_cast<int>(hw.major) << "."
				  << static_cast<int>(hw.minor) << '\n';

		std::cout << "FW: " << static_cast<int>(fw.major) << "."
				  << static_cast<int>(fw.minor) << '\n';
	}

	otos.setLinearUnit(kSfeOtosLinearUnitMeters);
	otos.setAngularUnit(kSfeOtosAngularUnitRadians);

	otos.resetTracking();

	std::cout << "Reading OTOS...\n";

	while (true) {
		sfe_otos_pose2d_t pos;
		sfe_otos_pose2d_t vel;
		sfe_otos_pose2d_t acc;

		const sfTkError_t err = otos.getPosVelAcc(pos, vel, acc);

		if (err != ksfTkErrOk) {
			std::cerr << "OTOS read error: " << static_cast<int>(err) << '\n';

			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			continue;
		}

		std::cout << "POS " << "x=" << pos.x << " m " << "y=" << pos.y << " m "
				  << "h=" << pos.h << " rad" << " | VEL " << "x=" << vel.x
				  << " m/s " << "y=" << vel.y << " m/s " << "h=" << vel.h
				  << " rad/s" << '\n';

		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
}