// sim_track -- closed-loop simulator that runs the REAL control stack.
//
// It drives the actual navigation::NavigationController and
// lidar::LidarProcessor (plus the real kinematics and logging) around a virtual
// WRO field, so the corner logic, wall detection, and the curvature/κ pipeline
// are exercised exactly as on the robot -- only the sensors and drivetrain are
// modelled. It writes the SAME telemetry.csv / corner_plan.csv / walls.csv /
// segments.csv / corners.csv as a real run, so plot_run.py, corner_baseline.py
// and analyze tools work on its output unchanged.
//
// What is modelled (the knobs the failures live in):
//   * LiDAR       -- ray-cast against the track walls at a real scan rate (Hz),
//                    with range noise and a one-scan actuation latency; the
//                    processor sees genuine geometry, not a fabricated scan.
//   * OTOS error  -- pose/velocity reported to the controller are scaled by
//                    --otos-scale (default 0.88 = the -12% linear error), while
//                    the LiDAR sees the TRUE geometry, reproducing the real
//                    sensor mismatch.
//   * drivetrain  -- command speed -> wheel RPM (with the app's scale + stall
//                    floor) -> actual speed, with a low-RPM stall and a lag, so
//                    the motor stall and the min-RPM floor are reproducible.
//   * vehicle     -- bicycle kinematics turned by the TRUE curvature_gain
//                    (--gain, default 2.0): the chassis pivots faster than the
//                    controller's model, which is what over-turns the corners.
//
// Usage:  sim_track [--gain G] [--otos-scale S] [--stall-rpm R] [--hz H]
//                   [--noise M] [--laps N] [--max-time T] [--seed K] [--quiet]
//
// It links only pure libraries (navigation, lidar_processor, logging, common) --
// no OTOS/SPI/camera hardware -- so it builds and runs on a dev machine.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "kinematics.hpp"
#include "lidar_processor.hpp"
#include "lidar_struct.hpp"
#include "navigation_controller.hpp"
#include "servo_pulse.hpp"
#include "track_map.hpp"

#include "corner_plan_logger.hpp"
#include "log_types.hpp"
#include "run_metadata.hpp"
#include "segment_logger.hpp"
#include "telemetry_logger.hpp"
#include "wall_logger.hpp"

#include "open_challenge_actuator.hpp"
#include "open_challenge_common.hpp"

namespace {

constexpr float PI = 3.14159265358979323846f;

struct Segment {
	float ax, ay, bx, by;
};

// WRO-style field: outer square (inner face at +/-outer_half) with a centred
// square block (outer face at +/-inner_half). The 1 m-ish gap between them is
// the driving corridor. Walls are the eight faces.
std::vector<Segment> build_track(float outer_half, float inner_half) {
	const float O = outer_half, I = inner_half;
	return {
		// outer walls (inward faces)
		{-O, -O, O, -O}, {O, -O, O, O}, {O, O, -O, O}, {-O, O, -O, -O},
		// inner block faces
		{-I, -I, I, -I}, {I, -I, I, I}, {I, I, -I, I}, {-I, I, -I, -I},
	};
}

// Nearest forward intersection distance of ray (px,py)+t*(dx,dy) with any
// segment, or -1 if none within max_range.
float ray_cast(float px, float py, float dx, float dy,
	const std::vector<Segment> &walls, float max_range) {
	float best = max_range;
	bool hit = false;
	for (const auto &s : walls) {
		const float ex = s.bx - s.ax, ey = s.by - s.ay;
		const float denom = dx * ey - dy * ex;
		if (std::abs(denom) < 1e-9f) {
			continue;
		}
		const float t = ((s.ax - px) * ey - (s.ay - py) * ex) / denom;
		const float u = ((s.ax - px) * dy - (s.ay - py) * dx) / denom;
		if (t > 1e-4f && t < best && u >= 0.0f && u <= 1.0f) {
			best = t;
			hit = true;
		}
	}
	return hit ? best : -1.0f;
}

// True vehicle pose. Frame matches the controller everywhere: forward at
// heading h is (-sin h, cos h) (so heading 0 faces +Y), right is (cos h, sin h),
// and heading is +CCW.
struct Pose {
	float x{0}, y{0}, h{0};
};

// Build one scan from the true pose. Raw angle a maps to a robot-frame ray
// direction (-sin a, -cos a) in (right, forward) -- the inverse of the
// processor's polar2cartesian -- so the resolved geometry is self-consistent.
TimedLidarData synth_scan(const Pose &p, const std::vector<Segment> &walls,
	int rays, float max_range, float noise_m, std::uint64_t timestamp_us,
	std::uint64_t scan_period_us, std::mt19937 &rng) {
	TimedLidarData scan;
	scan.timestamp_us = timestamp_us;
	scan.scan_period_us = scan_period_us;
	const float cos_h = std::cos(p.h), sin_h = std::sin(p.h);
	std::normal_distribution<float> noise(0.0f, noise_m);
	scan.points.reserve(rays);
	for (int i = 0; i < rays; ++i) {
		const float a = static_cast<float>(i) / rays * 360.0f;
		const float ar = a * PI / 180.0f;
		const float rr = -std::sin(ar), rf = -std::cos(ar); // robot right, fwd
		// robot(right,fwd) -> world using right=(cos h,sin h), fwd=(-sin h,cos h)
		const float dx = rr * cos_h + rf * (-sin_h);
		const float dy = rr * sin_h + rf * cos_h;
		float d = ray_cast(p.x, p.y, dx, dy, walls, max_range);
		if (d < 0.0f) {
			continue; // beyond range -> no return, like a real LiDAR
		}
		d += noise(rng);
		if (d < 0.05f || d > max_range) {
			continue;
		}
		LidarPoint pt;
		pt.angle_deg = a;
		pt.distance_m = d;
		pt.quality = 60;
		pt.scan_phase = static_cast<float>(i) / rays;
		scan.points.push_back(pt);
	}
	return scan;
}

struct SimConfig {
	float true_curvature_gain{2.0f};
	float otos_linear_scale{0.88f}; // -12% linear error
	float otos_angular_scale{1.0f};
	float stall_rpm{60.0f};	   // below this commanded RPM the drivetrain stalls
	float drivetrain_eff{1.0f}; // achieved / nominal wheel speed above stall
	float speed_lag_tau_s{0.08f};
	int rays{1440};
	float max_range_m{3.0f};
	float noise_m{0.003f};
	float scan_hz{15.0f};
	int latency_ticks{1};
	int max_laps{3};
	float max_time_s{120.0f};
	unsigned seed{1};
	bool quiet{false};
	float outer_half_m{1.5f};
	float inner_half_m{0.5f};
	// Start on a clean straight (left corridor, alongside the inner block), just
	// past the bottom-left corner, facing +Y -- not in a corner region, so the
	// first straight is clean and the corner triggers at the real corner.
	Pose start{-1.0f, -0.45f, 0.0f};
};

// Replicate ActuatorOutput::to_wheel_rpm (scale + stall floor + clamp) so the
// sim drivetrain sees exactly the command the robot would send.
float command_to_rpm(float speed_mps, const open_challenge::ActuatorConfig &a) {
	const float d = std::max(0.001f, a.wheel_diameter_m);
	float rpm = speed_mps * 60.0f / (PI * d) * a.motor_rpm_command_scale;
	if (a.minimum_moving_wheel_rpm > 0 && std::abs(speed_mps) > 1e-3f &&
		std::abs(rpm) < static_cast<float>(a.minimum_moving_wheel_rpm)) {
		rpm = std::copysign(
			static_cast<float>(a.minimum_moving_wheel_rpm), speed_mps);
	}
	const float max_rpm = static_cast<float>(a.maximum_wheel_rpm);
	return std::clamp(rpm, -max_rpm, max_rpm);
}

float parse_arg(const char *v, float dflt) {
	try {
		return std::stof(v);
	} catch (...) {
		return dflt;
	}
}

} // namespace

int main(int argc, char **argv) {
	SimConfig sim;
	for (int i = 1; i < argc; ++i) {
		const std::string o(argv[i]);
		auto next = [&](float d) { return i + 1 < argc ? parse_arg(argv[++i], d) : d; };
		if (o == "--gain") sim.true_curvature_gain = next(sim.true_curvature_gain);
		else if (o == "--otos-scale") sim.otos_linear_scale = next(sim.otos_linear_scale);
		else if (o == "--stall-rpm") sim.stall_rpm = next(sim.stall_rpm);
		else if (o == "--eff") sim.drivetrain_eff = next(sim.drivetrain_eff);
		else if (o == "--hz") sim.scan_hz = next(sim.scan_hz);
		else if (o == "--noise") sim.noise_m = next(sim.noise_m);
		else if (o == "--rays") sim.rays = static_cast<int>(next(sim.rays));
		else if (o == "--latency") sim.latency_ticks = static_cast<int>(next(sim.latency_ticks));
		else if (o == "--laps") sim.max_laps = static_cast<int>(next(sim.max_laps));
		else if (o == "--max-time") sim.max_time_s = next(sim.max_time_s);
		else if (o == "--seed") sim.seed = static_cast<unsigned>(next(sim.seed));
		else if (o == "--quiet") sim.quiet = true;
		else {
			std::cerr << "Usage: " << argv[0]
					  << " [--gain G] [--otos-scale S] [--stall-rpm R] [--hz H]"
						 " [--noise M] [--laps N] [--max-time T] [--seed K]"
						 " [--quiet]\n";
			return 2;
		}
	}

	// The real config the robot runs, so the sim reproduces its behaviour.
	navigation::NavigationConfig nav_config =
		open_challenge::make_navigation_config();
	nav_config.enable_replay_speed_factors = true;
	open_challenge::ActuatorConfig act_config;
	act_config.motor_rpm_command_scale = 1.0f;
	act_config.minimum_moving_wheel_rpm = 110;

	navigation::NavigationController navigation(nav_config);
	navigation::TrackMap track_map;
	lidar::LidarProcessor processor;

	const std::vector<Segment> walls =
		build_track(sim.outer_half_m, sim.inner_half_m);
	std::mt19937 rng(sim.seed);

	const float dt_s = 1.0f / std::max(1.0f, sim.scan_hz);
	const std::uint64_t dt_us = static_cast<std::uint64_t>(dt_s * 1e6f);

	const std::string run_dir = logging::make_run_directory("logs/sim");
	logging::JsonObject meta = logging::make_run_metadata(
		argv[0], nav_config, 1.0f / sim.otos_linear_scale, 1.0f);
	meta.add_object("actuator_config",
		open_challenge::actuator_config_json(act_config));
	{
		logging::JsonObject s;
		s.add_number("true_curvature_gain", sim.true_curvature_gain)
			.add_number("otos_linear_scale", sim.otos_linear_scale)
			.add_number("stall_rpm", sim.stall_rpm)
			.add_number("scan_hz", sim.scan_hz)
			.add_number("noise_m", sim.noise_m)
			.add_unsigned("latency_ticks",
				static_cast<std::uint64_t>(std::max(0, sim.latency_ticks)));
		meta.add_object("sim", s);
	}
	logging::write_run_metadata(run_dir, meta);
	logging::TelemetryLogger telemetry_log(run_dir);
	logging::WallLogger wall_log(run_dir);
	logging::SegmentLogger segment_log(run_dir);
	logging::CornerPlanLogger corner_plan_log(run_dir);
	std::cout << "sim -> " << run_dir << "  (gain=" << sim.true_curvature_gain
			  << ", otos_scale=" << sim.otos_linear_scale
			  << ", stall_rpm=" << sim.stall_rpm << ", hz=" << sim.scan_hz
			  << ")\n";

	Pose pose = sim.start;
	kinematics::BicycleModel true_model;
	true_model.wheelbase_m = nav_config.wheelbase_m;
	true_model.curvature_gain = sim.true_curvature_gain;

	float vehicle_speed = 0.0f;
	bool initialized = false;
	navigation::NavigationMode prev_mode =
		navigation::NavigationMode::SEARCH_DIRECTION;
	std::size_t turn_corner_index = 0;
	std::deque<navigation::NavigationCommand> command_delay;

	const std::uint64_t max_steps =
		static_cast<std::uint64_t>(sim.max_time_s / dt_s);
	std::uint64_t t_us = 0;
	bool finished = false;

	for (std::uint64_t step = 0; step < max_steps && !finished; ++step, t_us += dt_us) {
		// --- sense: LiDAR from TRUE pose, OTOS scaled from TRUE pose ---
		TimedLidarData scan =
			synth_scan(pose, walls, sim.rays, sim.max_range_m, sim.noise_m,
				t_us, dt_us, rng);

		const float otos_h = pose.h * sim.otos_angular_scale;
		const navigation::MapPose otos_pose{pose.x * sim.otos_linear_scale,
			pose.y * sim.otos_linear_scale, otos_h};
		const float otos_speed = vehicle_speed * sim.otos_linear_scale;

		if (!initialized) {
			navigation.reset(otos_h);
			prev_mode = navigation.state().mode;
			initialized = true;
		}

		// OTOS velocity in world frame (forward = (-sin h, cos h)).
		const float vx = otos_speed * -std::sin(otos_h);
		const float vy = otos_speed * std::cos(otos_h);

		const auto replay_hint =
			track_map.replay_hint(otos_pose, navigation.state().corner_index);
		const lidar::ScanMotion motion{otos_speed, 0.0f, dt_s, true};
		const lidar::ProcessedLidarData processed =
			open_challenge::process_scan(processor, scan, motion);

		navigation::NavigationResult result = navigation.update(
			processed, otos_h, otos_speed, replay_hint, otos_pose);
		const navigation::NavigationState &state = navigation.state();

		if (state.direction.has_value()) {
			track_map.set_direction(*state.direction);
		}
		// Corner map bookkeeping, mirroring the open main loop.
		if (prev_mode != navigation::NavigationMode::TURNING &&
			state.mode == navigation::NavigationMode::TURNING) {
			turn_corner_index = state.corner_index;
			const float trig = result.debug.effective_turn_trigger_m > 0.0f
				? result.debug.effective_turn_trigger_m
				: nav_config.turn_trigger_distance_m;
			track_map.record_corner_entry(state.corner_index,
				{otos_pose, trig, nav_config.corner_radius_m,
					result.command.target_speed_mps});
		}
		if (prev_mode == navigation::NavigationMode::TURNING &&
			state.mode != navigation::NavigationMode::TURNING) {
			track_map.record_corner_exit(turn_corner_index, otos_pose);
		}

		// --- log (same rows/format as a real run) ---
		const logging::OdometrySample odometry{t_us, vx, vy, 0.0f, 0.0f, 0.0f};
		logging::OutputSnapshot output;
		output.wheel_rpm =
			static_cast<std::int16_t>(command_to_rpm(
				result.command.target_speed_mps, act_config));
		const actuator::ServoPulseConfig servo{act_config.servo_min_pulse_us,
			act_config.servo_center_pulse_us, act_config.servo_max_pulse_us,
			act_config.maximum_servo_step_us, act_config.steering_to_servo_sign,
			act_config.maximum_steering_command_deg};
		output.servo_pulse_us =
			actuator::to_servo_pulse_us(servo, result.command.steering_rad);
		output.commanded_servo_pulse_us = output.servo_pulse_us;
		const logging::BatterySample battery{12.4f, 0, true};
		const logging::StageTiming timing{0, 0, false};
		telemetry_log.record(logging::make_telemetry_row(t_us, otos_pose,
			otos_speed, state, result, processed.obstacles.size(), output,
			odometry, battery, timing));
		wall_log.record(processed.walls, state.mode, otos_pose, t_us);
		segment_log.record(processed, state.mode);
		corner_plan_log.record(result, state, otos_pose, t_us);

		if (state.mode != prev_mode && !sim.quiet) {
			std::cout << "t=" << t_us / 1e6 << "s  "
					  << logging::navigation_mode_name(prev_mode) << " -> "
					  << logging::navigation_mode_name(state.mode)
					  << "  lap=" << state.lap << " corner=" << state.corner_index
					  << "  head=" << pose.h * 180.0f / PI << "deg\n";
		}
		prev_mode = state.mode;
		if (state.mode == navigation::NavigationMode::FINISHED) {
			finished = true;
		}

		// --- actuate: command delayed by latency_ticks, then drivetrain ---
		command_delay.push_back(result.command);
		navigation::NavigationCommand applied = command_delay.front();
		if (static_cast<int>(command_delay.size()) > sim.latency_ticks + 1) {
			command_delay.pop_front();
		} else {
			applied = navigation::NavigationCommand{}; // warm-up: no command yet
		}

		// Drivetrain: command speed -> rpm (scale+floor) -> actual speed, with a
		// low-RPM stall and a first-order lag.
		const float cmd_rpm =
			command_to_rpm(applied.target_speed_mps, act_config);
		float target_speed = 0.0f;
		if (std::abs(cmd_rpm) >= sim.stall_rpm) {
			target_speed = cmd_rpm / 60.0f * PI * act_config.wheel_diameter_m *
				sim.drivetrain_eff;
		} // else: stalled, target 0
		const float alpha =
			dt_s / std::max(1e-3f, sim.speed_lag_tau_s + dt_s);
		vehicle_speed += (target_speed - vehicle_speed) * alpha;

		// --- integrate TRUE pose with the TRUE curvature gain ---
		const float kappa =
			true_model.curvature_for_steering(applied.steering_rad);
		const float yaw_rate = -vehicle_speed * kappa; // +kappa=right=CW=-heading
		pose.h += yaw_rate * dt_s;
		pose.x += vehicle_speed * -std::sin(pose.h) * dt_s;
		pose.y += vehicle_speed * std::cos(pose.h) * dt_s;

		if (state.lap >= sim.max_laps + 1) {
			finished = true;
		}
	}

	logging::dump_corners(run_dir, track_map);
	telemetry_log.flush();
	wall_log.flush();
	segment_log.flush();
	corner_plan_log.flush();

	const navigation::NavigationState &s = navigation.state();
	std::cout << "\nDONE: " << (finished ? "finished/limit" : "timed out")
			  << "  turns=" << s.turn_count << " laps=" << s.lap
			  << "  final head=" << pose.h * 180.0f / PI << "deg\n"
			  << "Analyse: python3 code/tools/corner_baseline.py " << run_dir
			  << "\n";
	return 0;
}
