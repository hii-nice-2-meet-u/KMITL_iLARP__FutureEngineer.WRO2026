#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <optional>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "camera_module.hpp"
#include "camera_processor.hpp"
#include "obstacle_controller.hpp"
#include "obstacle_metadata.hpp"
#include "open_challenge_actuator.hpp"
#include "open_challenge_common.hpp"
#include "perception.hpp"
#include "run_metadata.hpp"
#include "telemetry_logger.hpp"
#include "track_map.hpp"
#include "wall_logger.hpp"

namespace {

std::atomic_bool stop_requested{false};

constexpr std::uint16_t SEARCH_LAUNCH_BOOST_RPM = 1500;
constexpr float SEARCH_LAUNCH_TIME_LIMIT_S = 0.3f;
constexpr float SEARCH_LAUNCH_RELEASE_SPEED_MPS = 0.18f;

void request_stop(int) {
	stop_requested.store(true);
	logging::notify_stop_requested();
}

} // namespace

int main(int, char **argv) {
	std::signal(SIGINT, request_stop);
	std::signal(SIGTERM, request_stop);

	navigation::NavigationConfig navigation_config =
		open_challenge::make_navigation_config();
	navigation_config.enable_replay_speed_factors = false;

	navigation_config.normal_speed_mps = 0.20f;
	navigation_config.approach_speed_mps = 0.17f;
	navigation_config.turning_speed_mps = 0.20f;
	navigation_config.search_speed_mps = 0.15f;
	navigation_config.maximum_replay_speed_mps = 0.42f;

	perception::PerceptionConfig perception_config;
	perception_config.lidar_mount = {0.0f, 0.081875f, 0.0f};
	perception_config.camera_mount = {0.0f, 0.0f, 0.0f};
	perception_config.max_sensor_time_difference_us = 100'000;
	perception_config.max_bearing_difference_rad =
		8.0f * 3.14159265358979323846f / 180.0f;
	perception_config.minimum_confirmed_confidence = 0.55f;

	obstacle_challenge::ObstacleConfig obstacle_config;
	obstacle_config.activation_distance_m = 1.50f;
	obstacle_config.pass_clearance_m = 0.32f;
	obstacle_config.minimum_lookahead_m = 0.35f;
	obstacle_config.avoidance_speed_mps = 0.25f;
	obstacle_config.maximum_avoidance_steering_rad =
		32.0f * 3.14159265358979323846f / 180.0f;
	obstacle_config.confirmation_frames = 1;
	obstacle_config.emergency_distance_m = 0.35f;
	obstacle_config.emergency_speed_mps = 0.16f;
	obstacle_config.emergency_steering_rad =
		32.0f * 3.14159265358979323846f / 180.0f;
	camera::CameraModule camera(640, 640, 90.0f, 1.8f, 2.8f);
	camera::CameraProcessor camera_processor;
	lidar::LidarModule lidar("/dev/ttyAMA0", 1'000'000);
	lidar::LidarProcessor lidar_processor;
	perception::Perception perception(perception_config);
	otos::OTOS otos;
	navigation::NavigationController navigation(navigation_config);
	navigation::TrackMap track_map;
	obstacle_challenge::ObstacleController obstacle_controller(obstacle_config);
	open_challenge::ActuatorConfig actuator_config;
	open_challenge::ActuatorOutput actuators(actuator_config);
	const std::uint64_t search_launch_time_limit_us =
		static_cast<std::uint64_t>(
			std::max(0.0f, SEARCH_LAUNCH_TIME_LIMIT_S) * 1'000'000.0f);

	std::cout << "Obstacle Challenge: camera + LiDAR fusion, OTOS map, SPI "
				 "active\n"
			  << "RED pass RIGHT, GREEN pass LEFT, confirmation="
			  << obstacle_config.confirmation_frames
			  << " frames, clearance=" << obstacle_config.pass_clearance_m
			  << "m\n";

	if (!camera.start()) {
		std::cerr << "Camera initialization failed\n";
		return 1;
	}
	if (!lidar.initialize() || !lidar.start()) {
		std::cerr << "LiDAR initialization failed\n";
		camera.stop();
		return 1;
	}
	if (!otos.initialize(1)) {
		std::cerr << "OTOS initialization failed\n";
		lidar.stop();
		camera.stop();
		return 1;
	}
	otos.setLinearUnit(kSfeOtosLinearUnitMeters);
	otos.setAngularUnit(kSfeOtosAngularUnitRadians);
	if (!open_challenge::calibrate_otos(otos)) {
		lidar.stop();
		camera.stop();
		return 1;
	}
	if (!actuators.initialize()) {
		std::cerr << "SPI actuator initialization failed\n";
		lidar.stop();
		camera.stop();
		return 1;
	}
	if (const auto voltage = actuators.get_voltage(); voltage.has_value()) {
		std::cout << "Battery voltage: " << *voltage << " V\n";
		if (*voltage <= 11.67f) {
			std::cerr << "Battery voltage is below 11.67 V\n";
			actuators.close();
			lidar.stop();
			camera.stop();
			return 1;
		}
	} else {
		std::cerr << "Battery voltage read failed\n";
	}

	const open_challenge::OtosScalars otos_scalars =
		open_challenge::read_otos_scalars(otos);
	const std::string run_directory = logging::make_run_directory();
	logging::JsonObject run_metadata = logging::make_run_metadata(argv[0],
		navigation_config, otos_scalars.linear, otos_scalars.angular);
	run_metadata
		.add_object("actuator_config",
			open_challenge::actuator_config_json(actuator_config))
		.add_object(
			"perception_config",
			obstacle_challenge::perception_config_json(perception_config))
		.add_object("obstacle_config",
			obstacle_challenge::obstacle_config_json(obstacle_config));
	if (!logging::write_run_metadata(run_directory, run_metadata)) {
		std::cerr << "Cannot write run_meta.json; refusing unattributed run\n";
		actuators.close();
		lidar.stop();
		camera.stop();
		return 1;
	}
	logging::TelemetryLogger telemetry_log(run_directory);
	logging::WallLogger wall_log(run_directory);
	std::optional<logging::EventLogger> event_log;
	std::cout << "Logging to " << run_directory << '\n';

	bool initialized = false;
	std::uint64_t launch_start_us = 0;
	bool launch_complete = false;
	std::uint64_t last_console_us = 0;
	std::uint64_t last_obstacle_capture_us = 0;
	navigation::NavigationMode previous_mode =
		navigation::NavigationMode::SEARCH_DIRECTION;

	while (!stop_requested.load()) {
		TimedLidarData scan;
		if (!lidar.wait_for_data(scan) || scan.points.empty()) {
			std::cerr << "LiDAR stream failed; emergency stop\n";
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

		if (!initialized) {
			navigation.reset(heading_rad);
			previous_mode = navigation.state().mode;
			launch_start_us = scan.timestamp_us;
			if (!actuators.arm()) {
				std::cerr << "Motor arm failed\n";
				break;
			}
			initialized = true;
		}
		if (!event_log.has_value()) {
			event_log.emplace(run_directory, scan.timestamp_us);
		}

		const navigation::MapPose pose{position.x, position.y, heading_rad};
		const auto replay_hint =
			track_map.replay_hint(pose, navigation.state().corner_index);
		const float wall_correction_rad = open_challenge::normalize_angle(
			heading_rad - navigation.state().target_heading_rad);
		const auto processed_lidar = open_challenge::process_scan(
			lidar_processor, scan, wall_correction_rad);

		TimedFrameData frame;
		camera::ProcessedCameraData processed_camera;
		if (camera.get_latest(frame) && !frame.frame.empty()) {
			processed_camera = camera_processor.process(frame);
		}
		const auto fused =
			perception.process(processed_lidar, processed_camera, pose);

		const bool obstacle_candidate_visible =
			!processed_lidar.obstacles.empty() || !processed_camera.objects.empty();
		const bool capture_ready = last_obstacle_capture_us == 0 ||
			scan.timestamp_us - last_obstacle_capture_us >= 500'000;
		if (obstacle_candidate_visible && capture_ready && !frame.frame.empty()) {
			cv::Mat capture = frame.frame.clone();
			for (const auto &object : processed_camera.objects) {
				const cv::Scalar color = object.color == camera::Color::Red
					? cv::Scalar(0, 0, 255)
					: cv::Scalar(0, 255, 0);
				cv::rectangle(capture, object.bounding_box, color, 3, cv::LINE_AA);
			}
			const auto &diagnostics = fused.diagnostics;
			const std::string status = "LIDAR=" +
				std::to_string(diagnostics.valid_lidar_count) + " CAMERA=" +
				std::to_string(diagnostics.valid_camera_count) + " MATCH=" +
				std::to_string(diagnostics.matched_count) + " CONFIRMED=" +
				std::to_string(diagnostics.frame_confirmed_count);
			cv::rectangle(capture, {0, 0, capture.cols, 42},
				cv::Scalar(0, 0, 0), -1);
			cv::putText(capture, status, {10, 29}, cv::FONT_HERSHEY_SIMPLEX,
				0.65, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
			const std::string capture_path = run_directory +
				"/obstacle_" + std::to_string(scan.timestamp_us) + ".jpg";
			if (cv::imwrite(capture_path, capture)) {
				std::cout << "[CAPTURE] " << capture_path << '\n';
				last_obstacle_capture_us = scan.timestamp_us;
			} else {
				std::cerr << "[CAPTURE] failed: " << capture_path << '\n';
			}
		}

		for (const auto &object : fused.obstacles) {
			if (!object.frame_confirmed || !object.world_position.has_value() ||
				!object.traffic_color.has_value() ||
				!object.required_pass_side.has_value()) {
				continue;
			}
			track_map.observe_traffic_light(object.world_position->x_m,
				object.world_position->y_m, *object.traffic_color,
				*object.required_pass_side, object.fusion_confidence);
		}

		navigation::NavigationCommand priority_command;
		priority_command.target_speed_mps = obstacle_config.avoidance_speed_mps;
		const auto obstacle_status = obstacle_controller.apply(fused, pose,
			navigation.state().mode, track_map.traffic_landmarks(),
			navigation.state().lap >= 1, priority_command);

		navigation::NavigationResult result;
		if (obstacle_status.active) {
			result.command = priority_command;
		} else {
			result = navigation.update(
				processed_lidar, heading_rad, speed_mps, replay_hint, pose);
		}
		const auto &state = navigation.state();
		std::optional<std::int16_t> wheel_rpm_override;

		if (!launch_complete) {
			const bool front_requires_slowdown =
				processed_lidar.walls.front.has_value() &&
				processed_lidar.walls.front->perpendicular_distance() <
					navigation_config.search_front_slowdown_distance_m;
			const std::uint64_t elapsed_us =
				scan.timestamp_us >= launch_start_us
				? scan.timestamp_us - launch_start_us
				: search_launch_time_limit_us;
			if (state.mode != navigation::NavigationMode::SEARCH_DIRECTION ||
				front_requires_slowdown ||
				speed_mps >= SEARCH_LAUNCH_RELEASE_SPEED_MPS ||
				elapsed_us >= search_launch_time_limit_us) {
				launch_complete = true;
			} else {
				wheel_rpm_override =
					static_cast<std::int16_t>(SEARCH_LAUNCH_BOOST_RPM);
			}
		}

		if (obstacle_status.active) {
			wheel_rpm_override.reset();
		}
		if (obstacle_status.activated) {
			event_log->event(scan.timestamp_us, state.lap, state.corner_index,
				"obstacle avoidance activated");
		}
		if (obstacle_status.released) {
			event_log->event(scan.timestamp_us, state.lap, state.corner_index,
				"obstacle avoidance released");
		}

		if (state.direction.has_value()) {
			track_map.set_direction(*state.direction);
		}
		if (state.mode != previous_mode) {
			event_log->event(scan.timestamp_us, state.lap, state.corner_index,
				std::string("MODE ") +
					logging::navigation_mode_name(previous_mode) + " -> " +
					logging::navigation_mode_name(state.mode));
		}
		if (previous_mode != navigation::NavigationMode::TURNING &&
			state.mode == navigation::NavigationMode::TURNING) {
			const float trigger_m = result.debug.effective_turn_trigger_m > 0.0f
				? result.debug.effective_turn_trigger_m
				: navigation_config.turn_trigger_distance_m;
			track_map.record_corner_entry(state.corner_index,
				{pose, trigger_m, navigation_config.corner_radius_m,
					result.command.target_speed_mps});
		}
		if (previous_mode == navigation::NavigationMode::TURNING &&
			state.mode != navigation::NavigationMode::TURNING) {
			const std::size_t completed =
				(state.corner_index + navigation::TRACK_CORNER_COUNT - 1) %
				navigation::TRACK_CORNER_COUNT;
			track_map.record_corner_exit(completed, pose);
		}

		const bool finished =
			state.mode == navigation::NavigationMode::FINISHED;
		if (finished) {
			actuators.emergency_stop();
		} else if (!actuators.apply(result.command, wheel_rpm_override)) {
			std::cerr << "SPI actuator command failed; emergency stop\n";
			break;
		}

		const auto &actuator_telemetry = actuators.telemetry();
		const logging::OutputSnapshot output{
			actuator_telemetry.wheel_rpm, actuator_telemetry.servo_pulse_us};
		logging::TelemetryRow telemetry_row =
			logging::make_telemetry_row(scan.timestamp_us, pose, speed_mps,
				state, result, fused.obstacles.size(), output, odometry);
		// Written every tick, including ticks where avoidance is inactive, so
		// an activation can be read against the surrounding navigation state.
		telemetry_row.obstacle_active = obstacle_status.active;
		telemetry_row.obstacle_color = obstacle_status.color.has_value()
			? (*obstacle_status.color == navigation::TrafficColor::RED ? 1 : 2)
			: 0;
		telemetry_row.obstacle_pass_side =
			obstacle_status.pass_side.has_value()
			? (*obstacle_status.pass_side == navigation::PassSide::RIGHT ? 1
																		 : 2)
			: 0;
		telemetry_row.obstacle_forward_m = obstacle_status.forward_m;
		telemetry_row.obstacle_right_m = obstacle_status.right_m;
		telemetry_row.obstacle_target_right_m =
			obstacle_status.target_right_m;
		telemetry_row.obstacle_steering_rad = obstacle_status.steering_rad;
		telemetry_row.obstacle_confidence = obstacle_status.confidence;
		telemetry_row.obstacle_world_x_m = obstacle_status.world_x_m;
		telemetry_row.obstacle_world_y_m = obstacle_status.world_y_m;
		telemetry_row.lidar_valid_count =
			static_cast<int>(fused.diagnostics.valid_lidar_count);
		telemetry_row.camera_valid_count =
			static_cast<int>(fused.diagnostics.valid_camera_count);
		telemetry_row.matched_count =
			static_cast<int>(fused.diagnostics.matched_count);
		telemetry_row.frame_confirmed_count =
			static_cast<int>(fused.diagnostics.frame_confirmed_count);
		telemetry_row.camera_time_synchronized =
			fused.diagnostics.camera_time_synchronized;
		telemetry_log.record(telemetry_row);
		wall_log.record(
			processed_lidar.walls, state.mode, pose, scan.timestamp_us);

		if (last_console_us == 0 ||
			scan.timestamp_us - last_console_us >= 250'000) {
			open_challenge::print_command(result, state, heading_rad, speed_mps,
				actuator_telemetry.wheel_rpm,
				actuator_telemetry.servo_pulse_us);
			const auto &d = fused.diagnostics;
			std::cout << "[FUSION] sync="
					  << (d.camera_time_synchronized ? "YES" : "NO")
					  << " lidar=" << d.valid_lidar_count
					  << " camera=" << d.valid_camera_count
					  << " confirmed=" << d.frame_confirmed_count << '\n';
			if (obstacle_status.active) {
				std::cout << "[OBSTACLE] "
						  << obstacle_challenge::color_name(
								 *obstacle_status.color)
						  << " pass="
						  << obstacle_challenge::side_name(
								 *obstacle_status.pass_side)
						  << " forward=" << obstacle_status.forward_m
						  << "m right=" << obstacle_status.right_m
						  << "m target_right=" << obstacle_status.target_right_m
						  << "m steering="
						  << obstacle_status.steering_rad * 180.0f /
						3.14159265358979323846f
						  << "deg\n";
			} else {
				std::cout << "[OBSTACLE] inactive landmarks="
						  << track_map.traffic_landmarks().size() << '\n';
			}
			last_console_us = scan.timestamp_us;
		}

		previous_mode = state.mode;
		if (finished) {
			std::cout << "Obstacle Challenge complete: 3 laps\n";
			break;
		}
	}

	const bool emergency_stop_ok = actuators.emergency_stop();
	const auto final_voltage = actuators.get_voltage();
	const bool stopped_safely = actuators.close() && emergency_stop_ok;
	lidar.stop();
	camera.stop();
	const bool corners_ok = logging::dump_corners(run_directory, track_map);
	const bool telemetry_ok = telemetry_log.flush();
	const bool walls_ok = wall_log.flush();
	const bool events_ok = event_log.has_value() ? event_log->flush() : true;
	if (!corners_ok || !telemetry_ok || !walls_ok || !events_ok) {
		std::cerr << "Logging write failure; run data may be incomplete\n";
	}
	if (telemetry_log.dropped_row_count() > 0 ||
		wall_log.dropped_row_count() > 0) {
		std::cerr << "Logging queue overflow: telemetry="
				  << telemetry_log.dropped_row_count()
				  << " walls=" << wall_log.dropped_row_count() << '\n';
	}
	if (final_voltage.has_value()) {
		std::cout << "Battery voltage after run: " << *final_voltage << " V\n";
	}
	if (!stopped_safely) {
		std::cerr << "SAFE STOP FAILED\n";
		return 1;
	}
	std::cout << "Stopped safely; M1/M2 RPM=0, brake active, servo centered\n";
	return 0;
}
