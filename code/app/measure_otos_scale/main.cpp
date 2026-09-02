// measure_otos_scale -- OTOS linear-scale bench check (HARDWARE_CHECKS M-3 / test A).
//
// Motors stay OFF. Push the robot a known straight distance by hand and this
// compares the distance OTOS reports against the tape-measured truth, then
// prints the linear scalar that would correct it.
//
// Why it matters: everything downstream trusts OTOS displacement/speed as
// ground truth. setLinearScalar is limited to [0.872, 1.127] (+/-12.7%), so if
// the correction this tool computes is inside that band the sensor is
// trustworthy and correctable -- which in turn proves the ~1.6x measured/target
// SPEED gap is a real drivetrain (STM32 RPM) error, not a sensor artefact. If
// the correction is OUTSIDE the band, the scalar cannot fix it and the mounting
// or a deeper problem must be found first.
//
// Usage:  measure_otos_scale [true_distance_m]     (default 2.0)
//   [g] reset tracking to zero (do this at the start line)
//   [m] mark: freeze the reading and print the result
//   [a] apply the suggested scalar to the device (then re-run to confirm ~1.0)
//   [q] quit

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <poll.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "otos.hpp"

namespace {

volatile std::sig_atomic_t running = 1;
void signal_handler(int) { running = 0; }

constexpr float SCALAR_MIN = 0.872f; // SparkFun OTOS setLinearScalar limits
constexpr float SCALAR_MAX = 1.127f;

class TerminalRawMode {
  public:
	TerminalRawMode() {
		if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &old_term_) != 0) {
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
	if (poll(&fd, 1, 0) <= 0 || !(fd.revents & POLLIN)) {
		return false;
	}
	return read(STDIN_FILENO, &key, 1) == 1;
}

void report(float true_distance_m, float reported_m, float current_scalar) {
	std::cout << "\n\n--- OTOS linear-scale result ---\n";
	std::cout << "true distance (tape):   " << true_distance_m << " m\n";
	std::cout << "OTOS reported distance: " << reported_m << " m\n";
	if (reported_m < 1e-4f) {
		std::cout << "reported distance ~0; push the robot first.\n";
		return;
	}
	const float correction = true_distance_m / reported_m;
	std::cout << "OTOS error:             " << (correction - 1.0f) * 100.0f
			  << " %  (reported/true = " << reported_m / true_distance_m
			  << ")\n";
	std::cout << "current linear scalar:  " << current_scalar << "\n";
	const float suggested = current_scalar * correction;
	std::cout << "suggested scalar:       " << suggested << "\n";
	if (suggested < SCALAR_MIN || suggested > SCALAR_MAX) {
		std::cout << "WARNING: suggested scalar is OUTSIDE the device limit ["
				  << SCALAR_MIN << ", " << SCALAR_MAX << "].\n"
				  << "  OTOS cannot correct an error this large -- the fault is "
					 "not the linear scalar.\n"
				  << "  Check the OTOS mounting height/offset before trusting "
					 "its distance or speed.\n";
	} else {
		std::cout << "OTOS is within the correctable band: it can be trusted as "
					 "ground truth.\n"
				  << "  Press [a] to apply " << suggested
				  << ", then re-run and confirm the error is ~0.\n"
				  << "  A trustworthy OTOS means the ~1.6x measured/target "
					 "SPEED gap is a real\n"
				  << "  drivetrain (STM32 RPM) error, not a sensor artefact.\n";
	}
	std::cout << "--------------------------------\n\n";
}

} // namespace

int main(int argc, char **argv) {
	float true_distance_m = 2.0f;
	if (argc > 1) {
		true_distance_m = std::strtof(argv[1], nullptr);
	}
	if (!(true_distance_m > 0.0f)) {
		std::cerr << "true_distance_m must be positive\n";
		return 2;
	}

	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);
	TerminalRawMode terminal;

	otos::OTOS otos;
	std::cout << "Initializing OTOS...\n";
	if (!otos.initialize(1)) {
		std::cerr << "OTOS initialize failed\n";
		return 1;
	}
	otos.setLinearUnit(kSfeOtosLinearUnitMeters);
	otos.setAngularUnit(kSfeOtosAngularUnitRadians);

	std::cout << "Keep the robot still: calibrating OTOS IMU...\n";
	if (otos.calibrateImu(255, true) != ksfTkErrOk) {
		std::cerr << "IMU calibration failed\n";
		return 1;
	}
	otos.resetTracking();

	float current_scalar = 1.0f;
	if (otos.getLinearScalar(current_scalar) != ksfTkErrOk) {
		std::cerr << "warning: getLinearScalar failed; assuming 1.0\n";
		current_scalar = 1.0f;
	}

	std::cout << "\nMotors are OFF. Procedure:\n"
			  << "  1. Place the robot at the start line.\n"
			  << "  2. Press [g] to zero tracking.\n"
			  << "  3. Push it straight forward exactly " << true_distance_m
			  << " m (tape measure).\n"
			  << "  4. Press [m] to read the result.\n"
			  << "Keys: [g] reset  [m] mark  [a] apply suggested  [q] quit\n\n";

	float suggested_scalar = current_scalar;
	bool have_suggestion = false;

	while (running) {
		char key;
		if (get_key(key)) {
			if (key == 'g' || key == 'G') {
				otos.resetTracking();
				std::cout << "\n[OTOS] tracking reset to zero\n";
			} else if (key == 'm' || key == 'M') {
				sfe_otos_pose2d_t pos{};
				sfe_otos_pose2d_t vel{};
				sfe_otos_pose2d_t acc{};
				if (otos.getPosVelAcc(pos, vel, acc) != ksfTkErrOk) {
					std::cerr << "\n[OTOS] read failed\n";
					continue;
				}
				const float reported_m = std::hypot(pos.x, pos.y);
				report(true_distance_m, reported_m, current_scalar);
				if (reported_m >= 1e-4f) {
					suggested_scalar = current_scalar *
						(true_distance_m / reported_m);
					have_suggestion = suggested_scalar >= SCALAR_MIN &&
						suggested_scalar <= SCALAR_MAX;
				}
			} else if (key == 'a' || key == 'A') {
				if (!have_suggestion) {
					std::cout << "\n[OTOS] no in-range suggestion to apply; "
								 "mark a run first\n";
				} else if (otos.setLinearScalar(suggested_scalar) ==
					ksfTkErrOk) {
					current_scalar = suggested_scalar;
					std::cout << "\n[OTOS] applied linear scalar "
							  << suggested_scalar
							  << "; press [g] then re-measure to confirm ~0 "
								 "error\n";
				} else {
					std::cerr << "\n[OTOS] setLinearScalar failed\n";
				}
			} else if (key == 'q' || key == 'Q') {
				break;
			}
		}

		sfe_otos_pose2d_t pos{};
		sfe_otos_pose2d_t vel{};
		sfe_otos_pose2d_t acc{};
		if (otos.getPosVelAcc(pos, vel, acc) == ksfTkErrOk) {
			const float displacement_m = std::hypot(pos.x, pos.y);
			std::cout << "\rdisplacement=" << displacement_m << " m  (x="
					  << pos.x << " y=" << pos.y << ")            "
					  << std::flush;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

	std::cout << "\nDone. Record the result in docs/audit/HARDWARE_CHECKS.md.\n";
	return 0;
}
