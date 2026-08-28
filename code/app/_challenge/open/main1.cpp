#include <atomic>
#include <csignal>
#include <cstdint>
#include <iostream>

#include "open_challenge_common.hpp"

namespace {

std::atomic_bool stop_requested{false};

void request_stop(int) { stop_requested.store(true); }

} // namespace

int main() {
	std::signal(SIGINT, request_stop);
	std::signal(SIGTERM, request_stop);

	lidar::LidarModule lidar("/dev/ttyAMA0", 1000000);
	lidar::LidarProcessor lidar_processor;
	otos::OTOS otos;
	navigation::NavigationController navigation(
		open_challenge::make_navigation_config());

	std::cout << "Open Challenge main1: LIVE navigation, NO MAP, NO SPI\n";

	if (!lidar.initialize() || !lidar.start()) {
		std::cerr << "LiDAR initialization failed\n";
		return 1;
	}

	if (!otos.initialize(1)) {
		std::cerr << "OTOS initialization failed\n";
		lidar.stop();
		return 1;
	}

	otos.setLinearUnit(kSfeOtosLinearUnitMeters);
	otos.setAngularUnit(kSfeOtosAngularUnitRadians);
	otos.resetTracking();

	bool navigation_initialized = false;
	std::uint64_t last_log_timestamp_us = 0;

	while (!stop_requested.load()) {
		TimedLidarData scan;
		if (!lidar.wait_for_data(scan)) {
			std::cerr << "LiDAR stream stopped\n";
			break;
		}
		if (scan.points.empty()) {
			continue;
		}

		sfe_otos_pose2d_t position{};
		sfe_otos_pose2d_t velocity{};
		sfe_otos_pose2d_t acceleration{};
		if (otos.getPosVelAcc(position, velocity, acceleration) != ksfTkErrOk) {
			std::cerr << "OTOS read failed; command suppressed\n";
			continue;
		}

		const float heading_rad = position.h;
		const float speed_mps = std::hypot(velocity.x, velocity.y);
		if (!navigation_initialized) {
			navigation.reset(heading_rad);
			navigation_initialized = true;
		}

		const float wall_correction_rad = open_challenge::normalize_angle(
			heading_rad - navigation.state().target_heading_rad);
		const auto processed = open_challenge::process_scan(
			lidar_processor, scan, wall_correction_rad);
		const auto result =
			navigation.update(processed, heading_rad, speed_mps);

		if (last_log_timestamp_us == 0 ||
			scan.timestamp_us - last_log_timestamp_us >= 250000) {
			open_challenge::print_command(
				result, navigation.state(), heading_rad, speed_mps);
			last_log_timestamp_us = scan.timestamp_us;
		}

		if (navigation.state().mode == navigation::NavigationMode::FINISHED) {
			std::cout << "Open Challenge complete: 12 turns [NO SPI]\n";
			break;
		}
	}

	lidar.stop();
	std::cout << "Stopped safely; no actuator command was sent\n";
	return 0;
}
