#include <atomic>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <optional>

#include "open_challenge_actuator.hpp"
#include "open_challenge_common.hpp"
#include "telemetry_logger.hpp"
#include "wall_logger.hpp"

namespace {

std::atomic_bool stop_requested{false};

void request_stop(int) {
	stop_requested.store(true);
	logging::notify_stop_requested();
}

}

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

	const std::string run_directory = logging::make_run_directory();
	logging::TelemetryLogger telemetry_log(run_directory);
	logging::WallLogger wall_log(run_directory);
	std::optional<logging::EventLogger> event_log;
	std::cout << "Logging to " << run_directory << '\n';

	bool navigation_initialized = false;
	std::uint64_t last_log_timestamp_us = 0;
	navigation::NavigationMode previous_mode =
		navigation::NavigationMode::SEARCH_DIRECTION;
	bool previous_heading_hold_active = false;

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
			previous_mode = navigation.state().mode;
			if (!actuators.arm()) {
				std::cerr << "Motor arm failed; M1 power stop attempted\n";
				break;
			}
			navigation_initialized = true;
			std::cout
				<< "Actuators armed after first valid LiDAR + OTOS frame\n";
		}

		if (!event_log.has_value()) {
			event_log.emplace(run_directory, scan.timestamp_us);
		}

		const float wall_correction_rad = open_challenge::normalize_angle(
			heading_rad - navigation.state().target_heading_rad);
		const auto processed = open_challenge::process_scan(
			lidar_processor, scan, wall_correction_rad);
		const auto result =
			navigation.update(processed, heading_rad, speed_mps);
		const auto &state = navigation.state();
		const navigation::MapPose map_pose{position.x, position.y, heading_rad};

		if (state.mode != previous_mode) {
			event_log->event(scan.timestamp_us, state.lap, state.corner_index,
				std::string("MODE ") +
					logging::navigation_mode_name(previous_mode) + " -> " +
					logging::navigation_mode_name(state.mode));
			previous_mode = state.mode;
		}

		if (result.debug.heading_hold_active != previous_heading_hold_active) {
			event_log->fault(scan.timestamp_us,
				result.debug.heading_hold_active
					? "outer wall lost, heading hold engaged"
					: "outer wall reacquired, heading hold released");
			previous_heading_hold_active = result.debug.heading_hold_active;
		}

		const bool finished =
			state.mode == navigation::NavigationMode::FINISHED;
		if (finished) {
			actuators.emergency_stop();
		} else if (!actuators.apply(result.command)) {
			std::cerr << "SPI actuator command failed; emergency stop\n";
			break;
		}
		const auto &telemetry = actuators.telemetry();
		const logging::OutputSnapshot output{
			telemetry.power_percent, telemetry.servo_pulse_us};
		telemetry_log.record(
			logging::make_telemetry_row(scan.timestamp_us, map_pose, speed_mps,
				state, result, processed.obstacles.size(), output));
		wall_log.record(
			processed.walls, state.mode, map_pose, scan.timestamp_us);

		if (finished) {
			open_challenge::print_command(result, state, heading_rad, speed_mps,
				telemetry.power_percent, telemetry.servo_pulse_us);
			std::cout << "Open Challenge complete: 12 turns; M1 power=0\n";
			break;
		}

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
	const bool telemetry_ok = telemetry_log.flush();
	const bool walls_ok = wall_log.flush();
	const bool events_ok = event_log.has_value() ? event_log->flush() : true;
	if (!telemetry_ok || !walls_ok || !events_ok) {
		std::cerr << "Logging write failure; run data may be incomplete\n";
	}
	if (telemetry_log.dropped_row_count() > 0 ||
		wall_log.dropped_row_count() > 0) {
		std::cerr << "Logging queue overflow: telemetry="
				  << telemetry_log.dropped_row_count()
				  << " walls=" << wall_log.dropped_row_count() << '\n';
	}
	std::cout << "Stopped safely; M1 power=0\n";
	return 0;
}
