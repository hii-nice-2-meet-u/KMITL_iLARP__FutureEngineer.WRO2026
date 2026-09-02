#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

#include "open_challenge_actuator.hpp"
#include "open_challenge_common.hpp"
#include "run_metadata.hpp"
#include "segment_logger.hpp"
#include "telemetry_logger.hpp"
#include "track_map.hpp"
#include "wall_logger.hpp"

namespace {

std::atomic_bool stop_requested{false};

constexpr std::uint16_t SEARCH_LAUNCH_BOOST_RPM = 1500;
constexpr float SEARCH_LAUNCH_TIME_LIMIT_S = 0.7f;
constexpr float SEARCH_LAUNCH_RELEASE_SPEED_MPS = 0.18f;

void request_stop(int) {
	stop_requested.store(true);
	logging::notify_stop_requested();
}

}

int main(int argc, char **argv) {
	bool direction_only = false;
	for (int index = 1; index < argc; ++index) {
		const std::string option(argv[index]);
		if (option == "--direction-only") {
			direction_only = true;
		} else {
			std::cerr << "Usage: " << argv[0] << " [--direction-only]\n";
			return 2;
		}
	}

	std::signal(SIGINT, request_stop);
	std::signal(SIGTERM, request_stop);

	navigation::NavigationConfig navigation_config =
		open_challenge::make_navigation_config();
	navigation_config.enable_replay_speed_factors = true;
	lidar::LidarModule lidar("/dev/ttyAMA0", 1000000);
	lidar::LidarProcessor lidar_processor;
	otos::OTOS otos;
	navigation::NavigationController navigation(navigation_config);
	navigation::TrackMap track_map;
	open_challenge::ActuatorConfig actuator_config;
	// STM32 turns commanded RPM into ~1.75x the intended ground speed; pre-scale
	// by 1/1.75 on the Pi. Back-calculated from a 1.60 logged speed ratio and
	// the -8.7% OTOS under-report; refine with one straight run after applying
	// the OTOS linear scalar (measure_otos_scale) and set scale = 1 / ratio.
	actuator_config.motor_rpm_command_scale = 0.571f;
	open_challenge::ActuatorOutput actuators(actuator_config);
	const std::uint64_t search_launch_time_limit_us =
		static_cast<std::uint64_t>(
			std::max(0.0f, SEARCH_LAUNCH_TIME_LIMIT_S) * 1'000'000.0f);

	std::cout
		<< "Open Challenge main2: LAP 1 LEARN, LAPS 2-3 REPLAY, SPI ACTIVE\n"
		<< "Drive=twin N20 (mirrored, common input; shafts M1/M2 turn "
		   "opposite for one command), wheel diameter="
		<< actuator_config.wheel_diameter_m
		<< "m, max=" << actuator_config.maximum_wheel_rpm
		<< " RPM, servo pulse=" << actuator_config.servo_min_pulse_us << "-"
		<< actuator_config.servo_max_pulse_us
		<< "us, max servo step=" << actuator_config.maximum_servo_step_us
		<< "us, center=" << actuator_config.servo_center_pulse_us
		<< "us, run=" << (direction_only ? "DIRECTION ONLY" : "FULL") << '\n';

	if (!lidar.initialize()) {
		std::cerr << "LiDAR initialization failed\n";
		return 1;
	}
	if (!lidar.start()) {
		std::cerr << "LiDAR start failed\n";
		return 1;
	}

	if (!otos.initialize(1)) {
		std::cerr << "OTOS initialization failed\n";
		lidar.stop();
		return 1;
	}

	otos.setLinearUnit(kSfeOtosLinearUnitMeters);
	otos.setAngularUnit(kSfeOtosAngularUnitRadians);
	if (!open_challenge::calibrate_otos(otos)) {
		lidar.stop();
		return 1;
	}

	if (!actuators.initialize()) {
		std::cerr << "SPI actuator initialization failed; M1/M2 stop "
					 "attempted\n";
		lidar.stop();
		return 1;
	}

	if (const auto voltage = actuators.get_voltage(); voltage.has_value()) {
		if (voltage <= 11.67f) {
			std::cerr << "[WARNING] Battery voltage is low :" << *voltage
					  << "V    NEEDTEAST 11.67 V\n";
			return 1;
		}
		std::cout << "Battery voltage: " << *voltage << " V\n";
	} else {
		std::cerr << "Battery voltage read failed\n";
	}

	const open_challenge::OtosScalars otos_scalars =
		open_challenge::read_otos_scalars(otos);
	const std::string run_directory = logging::make_run_directory();
	logging::JsonObject run_metadata = logging::make_run_metadata(argv[0],
		navigation_config, otos_scalars.linear, otos_scalars.angular);
	run_metadata.add_object("actuator_config",
		open_challenge::actuator_config_json(actuator_config));
	if (!logging::write_run_metadata(run_directory, run_metadata)) {
		std::cerr << "Cannot write run_meta.json; refusing unattributed run\n";
		actuators.close();
		lidar.stop();
		return 1;
	}
	logging::TelemetryLogger telemetry_log(run_directory);
	logging::WallLogger wall_log(run_directory);
	logging::SegmentLogger segment_log(run_directory);
	std::optional<logging::EventLogger> event_log;
	std::cout << "Logging to " << run_directory << '\n';

	bool navigation_initialized = false;
	std::uint64_t last_log_timestamp_us = 0;
	// Battery is read at ~1 Hz, not per tick: get_voltage() is a blocking SPI
	// round-trip. The last good reading is held between samples so every row
	// carries a voltage, with its age, rather than a gap.
	constexpr std::uint64_t BATTERY_SAMPLE_INTERVAL_US = 1'000'000;
	std::optional<float> last_battery_voltage_v;
	std::uint64_t last_battery_sample_us = 0;
	navigation::NavigationMode previous_mode =
		navigation::NavigationMode::SEARCH_DIRECTION;
	bool previous_heading_hold_active = false;
	std::string turn_source = "SEARCH";
	std::string follow_source = "SEARCH";
	float follow_outer_distance_m = 0.0f;
	float follow_inner_distance_m = 0.0f;
	float follow_error_m = 0.0f;
	bool turn_inner_distance_valid = false;
	float turn_inner_distance_m = 0.0f;
	float turn_inner_forward_m = 0.0f;
	float turn_trigger_distance_m = 0.0f;
	std::uint64_t search_launch_start_timestamp_us = 0;
	bool search_launch_boost_complete = false;
	bool search_launch_boost_active = false;

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

		// Same steady_clock epoch as TimedLidarData::timestamp_us, so the
		// logged scan-to-pose skew is a measurement, not an assumption.
		const std::uint64_t pose_timestamp_us = static_cast<std::uint64_t>(
			std::chrono::duration_cast<std::chrono::microseconds>(
				std::chrono::steady_clock::now().time_since_epoch())
				.count());
		const logging::OdometrySample odometry{pose_timestamp_us, velocity.x,
			velocity.y, velocity.h, acceleration.x, acceleration.y};

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
			search_launch_start_timestamp_us = scan.timestamp_us;
			if (!actuators.arm()) {
				std::cerr << "Motor arm failed; M1/M2 stop attempted\n";
				break;
			}
			navigation_initialized = true;
			std::cout
				<< "Actuators armed after first valid LiDAR + OTOS frame\n";
		}

		if (!event_log.has_value()) {
			event_log.emplace(run_directory, scan.timestamp_us);
		}

		const navigation::MapPose map_pose{position.x, position.y, heading_rad};
		const auto replay_hint =
			track_map.replay_hint(map_pose, navigation.state().corner_index);

		const float wall_correction_rad = open_challenge::normalize_angle(
			heading_rad - navigation.state().target_heading_rad);
		const auto lidar_process_start = std::chrono::steady_clock::now();
		const auto processed = open_challenge::process_scan(
			lidar_processor, scan, wall_correction_rad);
		logging::StageTiming stage_timing;
		stage_timing.lidar_process_us =
			static_cast<std::uint32_t>(
				std::chrono::duration_cast<std::chrono::microseconds>(
					std::chrono::steady_clock::now() - lidar_process_start)
					.count());
		// The open app has no camera stage; camera_process_valid stays false.
		auto result = navigation.update(
			processed, heading_rad, speed_mps, replay_hint, map_pose);
		const auto &state = navigation.state();
		std::optional<std::int16_t> wheel_rpm_override;
		if (!search_launch_boost_complete) {
			const bool front_requires_slowdown =
				processed.walls.front.has_value() &&
				processed.walls.front->perpendicular_distance() <
					navigation_config.search_front_slowdown_distance_m;
			const std::uint64_t launch_elapsed_us =
				scan.timestamp_us >= search_launch_start_timestamp_us
				? scan.timestamp_us - search_launch_start_timestamp_us
				: search_launch_time_limit_us;
			const bool release_boost =
				state.mode != navigation::NavigationMode::SEARCH_DIRECTION ||
				front_requires_slowdown ||
				speed_mps >= SEARCH_LAUNCH_RELEASE_SPEED_MPS ||
				launch_elapsed_us >= search_launch_time_limit_us;
			if (release_boost) {
				search_launch_boost_complete = true;
				if (search_launch_boost_active) {
					std::cout << "Search launch boost released at " << speed_mps
							  << " m/s\n";
				}
			} else {
				wheel_rpm_override =
					static_cast<std::int16_t>(SEARCH_LAUNCH_BOOST_RPM);
				if (!search_launch_boost_active) {
					std::cout << "Search launch boost active: "
							  << SEARCH_LAUNCH_BOOST_RPM << " RPM, limit "
							  << SEARCH_LAUNCH_TIME_LIMIT_S << " s\n";
				}
				search_launch_boost_active = true;
			}
		}
		if (state.mode == navigation::NavigationMode::NORMAL ||
			(previous_mode != navigation::NavigationMode::TURNING &&
				state.mode == navigation::NavigationMode::TURNING)) {
			if (result.debug.effective_turn_trigger_m > 0.0f) {
				turn_trigger_distance_m =
					result.debug.effective_turn_trigger_m;
			}
			if (result.debug.wall_corner_confirmed) {
				turn_source = "INNER_CORNER";
				turn_inner_distance_valid = true;
				turn_inner_forward_m = result.debug.wall_corner_forward_m;
				turn_inner_distance_m = std::hypot(
					result.debug.wall_corner_forward_m,
					result.debug.wall_corner_lateral_m);
			} else if (result.debug.front_wall_fallback_active) {
				turn_source = "FRONT_FALLBACK";
				turn_inner_distance_valid = false;
			} else {
				turn_source = "LEGACY_FRONT";
				turn_inner_distance_valid = false;
			}
		}
		if (state.mode == navigation::NavigationMode::NORMAL) {
			follow_outer_distance_m = result.debug.outer_distance_m;
			follow_inner_distance_m = result.debug.inner_distance_m;
			follow_error_m = result.debug.distance_error_m;
			if (result.debug.corridor_center_active) {
				follow_source = "CENTER";
			} else if (result.debug.outer_wall_valid) {
				follow_source = "OUTER";
			} else if (result.debug.heading_hold_active) {
				follow_source = "HEADING_HOLD";
			} else {
				follow_source = "LOST";
			}
		}

		if (state.mode != previous_mode) {
			event_log->event(scan.timestamp_us, state.lap, state.corner_index,
				std::string("MODE ") +
					logging::navigation_mode_name(previous_mode) + " -> " +
					logging::navigation_mode_name(state.mode));
		}

		if (result.debug.heading_hold_active != previous_heading_hold_active) {
			event_log->fault(scan.timestamp_us,
				result.debug.heading_hold_active
					? "outer wall lost, heading hold engaged"
					: "outer wall reacquired, heading hold released");
			previous_heading_hold_active = result.debug.heading_hold_active;
		}

		if (state.direction.has_value()) {
			track_map.set_direction(*state.direction);
		}
		const bool direction_only_complete =
			direction_only && state.direction.has_value();
		if (direction_only_complete) {
			event_log->event(scan.timestamp_us, state.lap, state.corner_index,
				std::string("direction detected: ") +
					open_challenge::direction_name(state.direction));
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
			event_log->event(scan.timestamp_us, state.lap, state.corner_index,
				"corner entry learned");
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
			event_log->event(scan.timestamp_us, state.lap, completed_corner,
				"corner exit learned");
			std::cout << "[MAP] corner " << completed_corner << " complete; "
					  << track_map.learned_corner_count() << "/4 learned\n";
			if (track_map.ready_for_replay() && state.lap == 1) {
				std::cout << "[MAP] REPLAY READY for laps 2-3\n";
			}
			turn_inner_distance_valid = false;
		}

		const bool finished =
			state.mode == navigation::NavigationMode::FINISHED;
		if (finished || direction_only_complete) {
			actuators.emergency_stop();
		} else if (!actuators.apply(result.command, wheel_rpm_override)) {
			std::cerr << "SPI actuator command failed; emergency stop\n";
			break;
		}
		const auto &telemetry = actuators.telemetry();
		const logging::OutputSnapshot output{telemetry.wheel_rpm,
			telemetry.servo_pulse_us, telemetry.commanded_servo_pulse_us};
		if (last_battery_sample_us == 0 ||
			scan.timestamp_us - last_battery_sample_us >=
				BATTERY_SAMPLE_INTERVAL_US) {
			if (const auto voltage = actuators.get_voltage();
				voltage.has_value()) {
				last_battery_voltage_v = *voltage;
				last_battery_sample_us = scan.timestamp_us;
			}
		}
		logging::BatterySample battery;
		if (last_battery_voltage_v.has_value()) {
			battery.voltage_v = *last_battery_voltage_v;
			battery.sample_age_us = scan.timestamp_us - last_battery_sample_us;
			battery.valid = true;
		}
		logging::TelemetryRow telemetry_row =
			logging::make_telemetry_row(scan.timestamp_us, map_pose, speed_mps,
				state, result, processed.obstacles.size(), output, odometry,
				battery, stage_timing);
		telemetry_row.lidar_points_total =
			static_cast<int>(processed.reject_stats.total);
		telemetry_row.lidar_points_rejected_quality =
			static_cast<int>(processed.reject_stats.rejected_quality);
		telemetry_row.lidar_points_rejected_range =
			static_cast<int>(processed.reject_stats.rejected_range);
		telemetry_row.wall_correction_rad = wall_correction_rad;
		telemetry_log.record(telemetry_row);
		wall_log.record(
			processed.walls, state.mode, map_pose, scan.timestamp_us);
		segment_log.record(processed, state.mode);

		if (direction_only_complete) {
			std::cout << "============================================================\n";
			open_challenge::print_command(result, state, heading_rad, speed_mps,
				telemetry.wheel_rpm, telemetry.servo_pulse_us);
			std::cout << "[TURN] source=" << turn_source << '\n';
			std::cout << "Direction detected: "
					  << open_challenge::direction_name(state.direction)
					  << "; M1/M2 RPM=0\n";
			break;
		}

		if (finished) {
			std::cout << "============================================================\n";
			open_challenge::print_command(result, state, heading_rad, speed_mps,
				telemetry.wheel_rpm, telemetry.servo_pulse_us);
			std::cout << "[TURN] source=" << turn_source << '\n';
			std::cout << "Open Challenge complete: 3 laps; M1/M2 RPM=0\n";
			break;
		}

		if (last_log_timestamp_us == 0 ||
			scan.timestamp_us - last_log_timestamp_us >= 250000) {
			std::cout << "============================================================\n";
			open_challenge::print_command(result, state, heading_rad, speed_mps,
				telemetry.wheel_rpm, telemetry.servo_pulse_us);
			const bool current_inner_visible =
				result.debug.wall_corner_candidate_valid ||
				result.debug.wall_corner_confirmed;
			const float current_inner_distance_m = std::hypot(
				result.debug.wall_corner_forward_m,
				result.debug.wall_corner_lateral_m);
			const float displayed_inner_forward_m = current_inner_visible
				? result.debug.wall_corner_forward_m
				: turn_inner_forward_m;
			const float displayed_inner_distance_m = current_inner_visible
				? current_inner_distance_m
				: turn_inner_distance_m;
			const bool displayed_inner_valid =
				current_inner_visible || turn_inner_distance_valid;
			const float displayed_trigger_m =
				result.debug.effective_turn_trigger_m > 0.0f
				? result.debug.effective_turn_trigger_m
				: turn_trigger_distance_m;
			std::cout << "[TURN] source=" << turn_source << " corner_forward="
					  << displayed_inner_forward_m << "m"
					  << " trigger=" << displayed_trigger_m
					  << "m\n";
			std::cout << "[FOLLOW] source=" << follow_source
					  << " outer=" << follow_outer_distance_m << "m"
					  << " inner=" << follow_inner_distance_m << "m"
					  << " error=" << follow_error_m << "m\n";
			if (displayed_inner_valid) {
				std::cout << "[INNER] "
						  << (current_inner_visible ? "VISIBLE" : "TURN_ENTRY")
						  << " distance=" << displayed_inner_distance_m << "m"
						  << " forward=" << displayed_inner_forward_m << "m"
						  << " confirmed="
						  << (result.debug.wall_corner_confirmed ? "YES" : "NO")
						  << '\n';
			} else {
				std::cout << "[INNER] NOT_FOUND\n";
			}
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
	}

	const bool emergency_stop_ok = actuators.emergency_stop();
	const auto final_battery_voltage = actuators.get_voltage();
	const bool stopped_safely = actuators.close() && emergency_stop_ok;
	lidar.stop();
	const bool corners_ok = logging::dump_corners(run_directory, track_map);
	const bool telemetry_ok = telemetry_log.flush();
	const bool walls_ok = wall_log.flush();
	const bool segments_ok = segment_log.flush();
	const bool events_ok = event_log.has_value() ? event_log->flush() : true;
	const std::size_t telemetry_dropped_rows =
		telemetry_log.dropped_row_count();
	const std::size_t walls_dropped_rows = wall_log.dropped_row_count();
	const std::size_t segments_dropped_rows = segment_log.dropped_row_count();
	const std::size_t events_dropped_rows =
		event_log.has_value() ? event_log->dropped_row_count() : 0;
	logging::JsonObject logging_summary;
	logging_summary
		.add_unsigned("telemetry_dropped_rows", telemetry_dropped_rows)
		.add_unsigned("walls_dropped_rows", walls_dropped_rows)
		.add_unsigned("segments_dropped_rows", segments_dropped_rows)
		.add_unsigned("events_dropped_rows", events_dropped_rows);
	run_metadata.add_object("logging", logging_summary);
	const bool metadata_ok =
		logging::write_run_metadata(run_directory, run_metadata);
	if (!corners_ok || !telemetry_ok || !walls_ok || !segments_ok ||
		!events_ok || !metadata_ok) {
		std::cerr << "Logging write failure; run data may be incomplete\n";
	}
	std::cout << "Logging dropped rows: telemetry=" << telemetry_dropped_rows
			  << " walls=" << walls_dropped_rows
			  << " segments=" << segments_dropped_rows
			  << " events=" << events_dropped_rows << '\n';
	if (telemetry_dropped_rows > 0 || walls_dropped_rows > 0 ||
		segments_dropped_rows > 0 || events_dropped_rows > 0) {
		std::cerr << "Logging queue overflow: telemetry="
				  << telemetry_dropped_rows << " walls=" << walls_dropped_rows
				  << " segments=" << segments_dropped_rows
				  << " events=" << events_dropped_rows << '\n';
	}
	if (!stopped_safely) {
		std::cerr << "SAFE STOP FAILED; verify M1/M2 and servo manually\n";
		return 1;
	}
	if (final_battery_voltage.has_value()) {
		std::cout << "Battery voltage after run: " << *final_battery_voltage
				  << " V\n";
	} else {
		std::cerr << "Battery voltage read after run failed\n";
	}
	std::cout << "Stopped safely; M1/M2 RPM=0, brake active, servo centered\n";
	return 0;
}
