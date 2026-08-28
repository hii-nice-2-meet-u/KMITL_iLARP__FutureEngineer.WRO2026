#include <atomic>
#include <csignal>
#include <cstdint>
#include <iostream>

#include "open_challenge_actuator.hpp"
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
	open_challenge::ActuatorConfig actuator_config;
	open_challenge::ActuatorOutput actuators(actuator_config);

	std::cout << "Open Challenge main1: LIVE navigation, NO MAP, SPI ACTIVE\n"
			  << "Drive=M1_POWER only, cap="
			  << actuator_config.maximum_drive_percent
			  << "%, servo pulse=" << actuator_config.servo_min_pulse_us << "-"
			  << actuator_config.servo_max_pulse_us << "us, center="
			  << (actuator_config.servo_min_pulse_us +
					 actuator_config.servo_max_pulse_us) /
			2 << "us, wheel steering=+/-"
			  << actuator_config.maximum_wheel_angle_deg << "deg\n";

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

	if (!actuators.initialize()) {
		std::cerr
			<< "SPI actuator initialization failed; M1 power stop attempted\n";
		lidar.stop();
		return 1;
	}

	bool navigation_initialized = false;
	std::uint64_t last_log_timestamp_us = 0;

	while (!stop_requested.load()) {
		TimedLidarData scan;
		if (!lidar.wait_for_data(scan)) {
			std::cerr << "LiDAR stream stopped\n";
			actuators.emergency_stop();
			break;
		}
		if (scan.points.empty()) {
			std::cerr << "LiDAR returned an empty scan; emergency stop\n";
			actuators.emergency_stop();
			break;
		}

		sfe_otos_pose2d_t position{};
		sfe_otos_pose2d_t velocity{};
		sfe_otos_pose2d_t acceleration{};
		if (otos.getPosVelAcc(position, velocity, acceleration) != ksfTkErrOk) {
			std::cerr << "OTOS read failed; emergency stop\n";
			actuators.emergency_stop();
			break;
		}

		const float heading_rad = position.h;
		const float speed_mps = std::hypot(velocity.x, velocity.y);
		if (!std::isfinite(heading_rad) || !std::isfinite(speed_mps)) {
			std::cerr << "OTOS returned a non-finite value; emergency stop\n";
			actuators.emergency_stop();
			break;
		}

		if (!navigation_initialized) {
			navigation.reset(heading_rad);
			if (!actuators.arm()) {
				std::cerr << "Motor arm failed; M1 power stop attempted\n";
				break;
			}
			navigation_initialized = true;
			std::cout
				<< "Actuators armed after first valid LiDAR + OTOS frame\n";
		}

		const float wall_correction_rad = open_challenge::normalize_angle(
			heading_rad - navigation.state().target_heading_rad);
		const auto processed = open_challenge::process_scan(
			lidar_processor, scan, wall_correction_rad);
		const auto result =
			navigation.update(processed, heading_rad, speed_mps);
		const auto &state = navigation.state();

		if (state.mode == navigation::NavigationMode::FINISHED) {
			actuators.emergency_stop();
			const auto &telemetry = actuators.telemetry();
			open_challenge::print_command(result, state, heading_rad, speed_mps,
				telemetry.power_percent, telemetry.servo_pulse_us);
			std::cout << "Open Challenge complete: 12 turns; M1 power=0\n";
			break;
		}

		if (!actuators.apply(result.command)) {
			std::cerr << "SPI actuator command failed; emergency stop\n";
			break;
		}
		const auto &telemetry = actuators.telemetry();

		if (last_log_timestamp_us == 0 ||
			scan.timestamp_us - last_log_timestamp_us >= 250000) {
			open_challenge::print_command(result, state, heading_rad, speed_mps,
				telemetry.power_percent, telemetry.servo_pulse_us);
			last_log_timestamp_us = scan.timestamp_us;
		}
	}

	actuators.emergency_stop();
	actuators.close();
	lidar.stop();
	std::cout << "Stopped safely; M1 power=0\n";
	return 0;
}
