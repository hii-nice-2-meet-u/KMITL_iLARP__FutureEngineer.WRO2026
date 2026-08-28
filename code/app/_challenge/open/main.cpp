#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <linux/spi/spidev.h>
#include <ostream>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>


int main() {

	// =========================================================================
	// HARDWARE
	// =========================================================================

	lidar::LidarModule lidar("/dev/ttyAMA0", 1000000);

	lidar::LidarProcessor lidar_processor;

	otos::OTOS otos;

	// =========================================================================
	// NAVIGATION CONFIG
	// =========================================================================

	navigation::NavigationConfig nav_config;

	// =====================================================================
	// FIELD TUNING — change only these values during track testing.
	// Everything else has a stable default in its owning module.
	// =====================================================================

	// Path and speed profile
	nav_config.target_outer_distance_m = 0.30f;
	nav_config.normal_speed_mps = 0.85f;
	nav_config.turning_speed_mps = 0.65f;
	nav_config.approach_distance_m = 0.90f;

	// Stanley-PID steering: k corrects wall distance; PID corrects heading.
	nav_config.stanley.k = 0.85f;
	nav_config.stanley.heading_pid.kp = 1.00f;
	nav_config.stanley.heading_pid.ki = 0.12f;
	nav_config.stanley.heading_pid.kd = 0.025f;

	// Speed PID output is requested acceleration [m/s^2]. It requires actual
	// speed from an encoder when a motor controller is connected.
	nav_config.speed_pid.kp = 4.0f;
	nav_config.speed_pid.ki = 1.0f;
	nav_config.speed_pid.kd = 0.04f;

	navigation::NavigationController navigation(nav_config);

	// =========================================================================
	// LIDAR
	// =========================================================================

	std::cout << "Initializing LiDAR...\n";

	if (!lidar.initialize()) {

		std::cerr << "LiDAR initialize failed\n";

		return 1;
	}

	if (!lidar.start()) {

		std::cerr << "LiDAR start failed\n";

		return 1;
	}

	// =========================================================================
	// OTOS
	// =========================================================================

	std::cout << "Initializing OTOS...\n";

	if (!otos.initialize(1)) {

		std::cerr << "OTOS initialize failed\n";

		lidar.stop();

		return 1;
	}

	otos.setLinearUnit(kSfeOtosLinearUnitMeters);

	otos.setAngularUnit(kSfeOtosAngularUnitRadians);

	otos.resetTracking();

	std::cout << "LiDAR + OTOS ready\n";

	// =========================================================================
	// DEBUG WINDOW
	// =========================================================================

	cv::Mat debug_map(MAP_HEIGHT, MAP_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

	bool nav_initialized = false;

	std::uint64_t previous_timestamp_us = 0;

	navigation::NavigationMode previous_mode =
		navigation::NavigationMode::SEARCH_DIRECTION;

	std::optional<DrivingDirection> previous_direction;

	// =========================================================================
	// LOOP
	// =========================================================================

	while (true) {

		TimedLidarData scan;

		sfe_otos_pose2d_t pos{};
		sfe_otos_pose2d_t vel{};
		sfe_otos_pose2d_t acc{};

		// ---------------------------------------------------------------------
		// LIDAR
		// ---------------------------------------------------------------------

		if (!lidar.wait_for_data(scan)) {

			break;
		}

		if (scan.points.empty()) {
			continue;
		}

		// ---------------------------------------------------------------------
		// OTOS
		// ---------------------------------------------------------------------

		const sfTkError_t otos_error = otos.getPosVelAcc(pos, vel, acc);

		if (otos_error != ksfTkErrOk) {

			std::cerr << "OTOS read error: " << static_cast<int>(otos_error)
					  << '\n';

			continue;
		}

		const float heading_rad = pos.h;

		// Magnitude is enough for Stanley test.
		const float speed_mps = std::hypot(vel.x, vel.y);

		// ---------------------------------------------------------------------
		// INIT NAV
		// ---------------------------------------------------------------------

		if (!nav_initialized) {

			navigation.reset(heading_rad);

			previous_mode = navigation.state().mode;

			nav_initialized = true;

			std::cout << "[NAV] initialized heading="
					  << heading_rad * RAD_TO_DEG << " deg\n";
		}

		const float wall_correction_rad = normalize_angle(
			heading_rad - navigation.state().target_heading_rad);

		const auto process_start = std::chrono::steady_clock::now();

		const auto processed =
			lidar_processor.process(scan, wall_correction_rad,
				4,		// min_segment_point
				0.035f, // max_line_error_m
				0.12f,	// max_point_gap_m
				5.0f,	// max_angle_diff deg
				0.04f,	// max_collinear_error_m
				0.10f	// max_segment_gap_m
			);

		// ---------------------------------------------------------------------
		// REAL NavigationController
		// ---------------------------------------------------------------------

		const auto nav_result =
			navigation.update(processed, heading_rad, speed_mps);

		const auto process_us =
			std::chrono::duration_cast<std::chrono::microseconds>(
				process_end - process_start)
				.count();

		const auto &state = navigation.state();
	}

	// =========================================================================
	// STOP
	// =========================================================================

	std::cout << "\nStopping...\n";

	lidar.stop();

	return 0;
}