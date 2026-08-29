#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <poll.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "otos.hpp"

namespace {

volatile std::sig_atomic_t running = 1;

void signal_handler(int) { running = 0; }

class TerminalRawMode {
  public:
	TerminalRawMode() {
		if (!isatty(STDIN_FILENO)) {
			return;
		}

		if (tcgetattr(STDIN_FILENO, &old_term_) != 0) {
			return;
		}

		termios new_term = old_term_;

		new_term.c_lflag &= ~(ICANON | ECHO);

		new_term.c_cc[VMIN] = 0;
		new_term.c_cc[VTIME] = 0;

		if (tcsetattr(STDIN_FILENO, TCSANOW, &new_term) == 0) {
			active_ = true;
		}
	}

	~TerminalRawMode() {
		if (active_) {
			tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
		}
	}

  private:
	termios old_term_{};
	bool active_{false};
};

bool get_key(char &key) {
	pollfd fd{};
	fd.fd = STDIN_FILENO;
	fd.events = POLLIN;

	const int result = poll(&fd, 1, 0);

	if (result <= 0) {
		return false;
	}

	if (!(fd.revents & POLLIN)) {
		return false;
	}

	return read(STDIN_FILENO, &key, 1) == 1;
}

}

int main() {
	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);

	TerminalRawMode terminal;

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

	std::cout << '\n';
	std::cout << "Controls:\n";
	std::cout << "  [g] Reset tracking\n";
	std::cout << "  [h] Calibrate IMU\n";
	std::cout << "  [q] Quit\n";
	std::cout << '\n';

	while (running) {

		char key;

		if (get_key(key)) {

			if (key == 'g' || key == 'G') {

				const sfTkError_t err = otos.resetTracking();

				if (err == ksfTkErrOk) {
					std::cout << "\n[OTOS] Tracking reset\n";
				} else {
					std::cerr << "\n[OTOS] resetTracking failed: "
							  << static_cast<int>(err) << '\n';
				}

			} else if (key == 'h' || key == 'H') {

				std::cout << "\n[OTOS] Calibrating IMU...\n";

				const sfTkError_t err = otos.calibrateImu(255, true);

				if (err == ksfTkErrOk) {
					std::cout << "[OTOS] IMU calibration complete\n";
				} else {
					std::cerr << "[OTOS] IMU calibration failed: "
							  << static_cast<int>(err) << '\n';
				}

			} else if (key == 'q' || key == 'Q') {

				std::cout << "\nExiting...\n";
				break;
			}
		}

		sfe_otos_pose2d_t pos;
		sfe_otos_pose2d_t vel;
		sfe_otos_pose2d_t acc;

		const sfTkError_t err = otos.getPosVelAcc(pos, vel, acc);

		if (err != ksfTkErrOk) {

			std::cerr << "OTOS read error: " << static_cast<int>(err) << '\n';

			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			continue;
		}

		std::cout << "\rPOS " << "x=" << pos.x << " m " << "y=" << pos.y
				  << " m " << "h=" << pos.h << " rad" << " | VEL "
				  << "x=" << vel.x << " m/s " << "y=" << vel.y << " m/s "
				  << "h=" << vel.h << " rad/s" << "          " << std::flush;

		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

	std::cout << '\n';

	return 0;
}