#include <atomic>
#include <csignal>
#include <cstdint>
#include <iostream>

#include "open_challenge_common.hpp"
#include "track_map.hpp"

namespace {

std::atomic_bool stop_requested{false};

void request_stop(int) { stop_requested.store(true); }

} // namespace

int main() {
	std::signal(SIGINT, request_stop);
	std::signal(SIGTERM, request_stop);

	const navigation::NavigationConfig navigation_config =
		open_challenge::make_navigation_config();
	lidar::LidarModule lidar("/dev/ttyAMA0", 1000000);
	lidar::LidarProcessor lidar_processor;
	otos::OTOS otos;
	navigation::NavigationController navigation(navigation_config);
	navigation::TrackMap track_map;

	std::cout << "Open Challenge main2: LAP 1 LEARN, LAPS 2-3 REPLAY, NO SPI\n";

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
	navigation::NavigationMode previous_mode =
		navigation::NavigationMode::SEARCH_DIRECTION;

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
			previous_mode = navigation.state().mode;
			navigation_initialized = true;
		}

		const navigation::MapPose map_pose{position.x, position.y, heading_rad};
		const auto replay_hint =
			track_map.replay_hint(map_pose, navigation.state().corner_index);

		const float wall_correction_rad = open_challenge::normalize_angle(
			heading_rad - navigation.state().target_heading_rad);
		const auto processed = open_challenge::process_scan(
			lidar_processor, scan, wall_correction_rad);
		const auto result =
			navigation.update(processed, heading_rad, speed_mps, replay_hint);
		const auto &state = navigation.state();

		if (state.direction.has_value()) {
			track_map.set_direction(*state.direction);
		}

		if (previous_mode != navigation::NavigationMode::TURNING &&
			state.mode == navigation::NavigationMode::TURNING) {
			const float learned_trigger_m =
				result.debug.effective_turn_trigger_m > 0.0f
				? result.debug.effective_turn_trigger_m
				: navigation_config.turn_trigger_distance_m;
			track_map.record_corner_entry(state.corner_index,
				{map_pose, learned_trigger_m, navigation_config.corner_radius_m,
					result.command.target_speed_mps});
			std::cout << "[MAP] corner " << state.corner_index
					  << " entry x=" << position.x << " y=" << position.y
					  << '\n';
		}

		if (previous_mode == navigation::NavigationMode::TURNING &&
			state.mode != navigation::NavigationMode::TURNING) {
			const std::size_t completed_corner =
				(state.corner_index + navigation::TRACK_CORNER_COUNT - 1) %
				navigation::TRACK_CORNER_COUNT;
			track_map.record_corner_exit(completed_corner, map_pose);
			std::cout << "[MAP] corner " << completed_corner << " complete; "
					  << track_map.learned_corner_count() << "/4 learned\n";
			if (track_map.ready_for_replay() && state.lap == 1) {
				std::cout << "[MAP] REPLAY READY for laps 2-3\n";
			}
		}

		if (last_log_timestamp_us == 0 ||
			scan.timestamp_us - last_log_timestamp_us >= 250000) {
			open_challenge::print_command(
				result, state, heading_rad, speed_mps);
			if (replay_hint.has_value()) {
				std::cout << "[MAP] next=" << replay_hint->corner_index
						  << " distance=" << replay_hint->distance_to_entry_m
						  << "m confidence=" << replay_hint->confidence
						  << (replay_hint->approach_recommended
									 ? " MAP_APPROACH"
									 : "")
						  << '\n';
			}
			last_log_timestamp_us = scan.timestamp_us;
		}

		previous_mode = state.mode;
		if (state.mode == navigation::NavigationMode::FINISHED) {
			std::cout << "Open Challenge complete: 3 laps [NO SPI]\n";
			break;
		}
	}

	lidar.stop();
	std::cout << "Stopped safely; no actuator command was sent\n";
	return 0;
}
