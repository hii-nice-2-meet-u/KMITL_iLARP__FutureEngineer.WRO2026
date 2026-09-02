// measure -- automated hardware-characterisation bench harness.
//
// One binary, four sub-modes, each of which commands the vehicle through a
// scripted routine and writes a CSV so the numbers that block the motion model
// (M-4 curvature_gain, M-7 speed model, M-8 coast-down, M-11 LiDAR noise) come
// from repeatable machine measurements instead of by hand.
//
//   --speed-sweep    M-7 + M-4: grid of raw wheel RPM x steering angle. Holds
//                    each cell, logs steady-state OTOS speed / yaw rate /
//                    acceleration. Straight cells give the command->speed ratio
//                    (STM32 RPM-scale error); turning cells give curvature_gain.
//   --coast-down     M-8: spin up straight, disable the motors (true free roll,
//                    NOT brake), log the deceleration to rest.
//   --lidar-noise    M-11: motors off, capture N scans, log per-scan wall
//                    distances and point-rejection tallies (sensor noise floor).
//   --lidar-capture  Motors off, hand-push the robot; dump raw scan points plus
//                    the OTOS motion per scan so analyze_measure.py can render
//                    the before/after deskew comparison.
//
// This talks to spi::SPI and otos::OTOS directly rather than through the
// open-challenge ActuatorOutput: the bench needs raw RPM commands and a real
// coast (M_DISABLE), and the challenge actuator deliberately exposes neither.
//
// Safety: SIGINT/SIGTERM brake the motors and centre the servo before exit; a
// countdown precedes every routine that moves the wheels; any SPI/OTOS failure
// mid-routine brakes and aborts. The OTOS error we measured on this unit is
// corrected at startup (--otos-scalar, default 1.0956, clamped to the device
// band) so the logged speed and distance are trustworthy ground truth.
//
// Analyse the CSVs with code/tools/analyze_measure.py.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cctype>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <poll.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "kinematics.hpp"
#include "lidar_module.hpp"
#include "lidar_processor.hpp"
#include "otos.hpp"
#include "servo_pulse.hpp"
#include "spi_master.hpp"

namespace {

volatile std::sig_atomic_t g_running = 1;
void signal_handler(int) { g_running = 0; }

// SparkFun OTOS setLinearScalar hardware limits.
constexpr float SCALAR_MIN = 0.872f;
constexpr float SCALAR_MAX = 1.127f;

constexpr float PI = 3.14159265358979323846f;

std::uint64_t now_us() {
	return static_cast<std::uint64_t>(
		std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::steady_clock::now().time_since_epoch())
			.count());
}

// Sleep in short slices so SIGINT is honoured promptly during long holds.
// Returns false if a stop was requested while waiting.
bool interruptible_sleep_s(float seconds) {
	const std::uint64_t deadline = now_us() +
		static_cast<std::uint64_t>(std::max(0.0f, seconds) * 1e6f);
	while (g_running && now_us() < deadline) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	return g_running != 0;
}

// Puts the terminal in non-canonical, no-echo mode for the tool's lifetime so
// single keypresses (a/q) are read without waiting for Enter. Restores the old
// mode on destruction. A no-op if stdin is not a TTY.
class TerminalRawMode {
  public:
	TerminalRawMode() {
		if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &old_term_) != 0) {
			return;
		}
		termios raw = old_term_;
		raw.c_lflag &= ~(ICANON | ECHO);
		raw.c_cc[VMIN] = 0;
		raw.c_cc[VTIME] = 0;
		if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
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

bool poll_key(char &key) {
	pollfd fd{};
	fd.fd = STDIN_FILENO;
	fd.events = POLLIN;
	if (poll(&fd, 1, 0) <= 0 || !(fd.revents & POLLIN)) {
		return false;
	}
	return read(STDIN_FILENO, &key, 1) == 1;
}

// Block until one of `accept` is pressed (case-insensitive), returning the
// lowercased char. Returns 0 if interrupted (SIGINT/SIGTERM). If stdin is not a
// TTY the gate is skipped and the first accepted char is returned so the tool
// still runs non-interactively.
char wait_for_key(const std::string &accept) {
	if (!isatty(STDIN_FILENO)) {
		return accept.empty() ? 0 : accept.front();
	}
	while (g_running) {
		char key = 0;
		if (poll_key(key)) {
			const char lowered =
				static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
			if (accept.find(lowered) != std::string::npos) {
				return lowered;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return 0;
}

std::vector<float> parse_float_list(const std::string &text) {
	std::vector<float> values;
	std::stringstream stream(text);
	std::string token;
	while (std::getline(stream, token, ',')) {
		if (token.empty()) {
			continue;
		}
		try {
			values.push_back(std::stof(token));
		} catch (const std::exception &) {
			// Skip malformed tokens; the caller validates the resulting list.
		}
	}
	return values;
}

enum class Mode { NONE, SPEED_SWEEP, COAST_DOWN, LIDAR_NOISE, LIDAR_CAPTURE };

struct Config {
	Mode mode{Mode::NONE};

	// OTOS
	float otos_scalar{1.0956f}; // correction measured on this unit (true/report)

	// Vehicle geometry (measurements, kept in sync with kinematics.hpp).
	float wheel_diameter_m{0.0525f};
	float wheelbase_m{0.16375f};
	std::uint16_t max_wheel_rpm{1500};

	// Speed sweep grid. Within the competition operating range (<=300 RPM).
	std::vector<float> sweep_rpms{50, 75, 100, 125};
	std::vector<float> sweep_steer_deg{0};
	float dwell_s{2.5f};   // hold time per cell
	float settle_s{1.0f};  // leading slice the analyser discards (transient)
	float pause_s{2.0f};   // stopped gap between cells (decel captured too)
	float max_speed_mps{0.4f}; // hard OTOS speed limit for this 2 m test lane
	float distance_limit_m{2.0f}; // stop a cell before it can leave the test lane

	// Coast-down. At the competition speed (<=300 RPM), not beyond.
	float coast_rpm{125.0f};
	float coast_stop_speed_mps{0.03f};
	float coast_timeout_s{6.0f};

	// LiDAR.
	int scans{200};              // lidar-noise
	int capture_scans{20};       // lidar-capture
	std::string lidar_port{"/dev/ttyAMA0"};

	// Servo mapping (matches open_challenge::ActuatorConfig defaults).
	actuator::ServoPulseConfig servo{900, 1475, 2100, 500, 1.0f, 45.0f};

	std::string out_dir; // resolved run directory
};

// ---- OTOS -----------------------------------------------------------------

struct OtosSample {
	std::uint64_t t_us{0};
	float x{0}, y{0}, h{0};
	float vx{0}, vy{0}, speed{0}, yaw_rate{0};
	float ax{0}, ay{0};
};

bool read_otos(otos::OTOS &otos, OtosSample &out) {
	sfe_otos_pose2d_t pos{};
	sfe_otos_pose2d_t vel{};
	sfe_otos_pose2d_t acc{};
	if (otos.getPosVelAcc(pos, vel, acc) != ksfTkErrOk) {
		return false;
	}
	out.t_us = now_us();
	out.x = pos.x;
	out.y = pos.y;
	out.h = pos.h;
	out.vx = vel.x;
	out.vy = vel.y;
	out.speed = std::hypot(vel.x, vel.y);
	out.yaw_rate = vel.h;
	out.ax = acc.x;
	out.ay = acc.y;
	return true;
}

bool setup_otos(otos::OTOS &otos, const Config &config, float &applied_scalar) {
	std::cout << "Initializing OTOS...\n";
	if (!otos.initialize(1)) {
		std::cerr << "OTOS initialize failed\n";
		return false;
	}
	otos.setLinearUnit(kSfeOtosLinearUnitMeters);
	otos.setAngularUnit(kSfeOtosAngularUnitRadians);

	std::cout << "Keep the robot still: calibrating OTOS IMU...\n";
	if (otos.calibrateImu(255, true) != ksfTkErrOk) {
		std::cerr << "OTOS IMU calibration failed\n";
		return false;
	}

	const float requested = config.otos_scalar;
	const float clamped = std::clamp(requested, SCALAR_MIN, SCALAR_MAX);
	if (std::abs(clamped - requested) > 1e-4f) {
		std::cerr << "warning: --otos-scalar " << requested
				  << " is outside the device band [" << SCALAR_MIN << ", "
				  << SCALAR_MAX << "]; clamping to " << clamped << '\n';
	}
	applied_scalar = clamped;
	if (otos.setLinearScalar(clamped) != ksfTkErrOk) {
		std::cerr << "warning: setLinearScalar failed; logged distances will "
					 "carry the raw OTOS error\n";
		applied_scalar = 1.0f;
	} else {
		std::cout << "Applied OTOS linear scalar " << clamped
				  << " (measured correction for this unit)\n";
	}

	if (otos.resetTracking() != ksfTkErrOk) {
		std::cerr << "OTOS resetTracking failed\n";
		return false;
	}
	return true;
}

// Recalibrate the IMU and re-zero tracking on a device already brought up by
// setup_otos(). The linear scalar persists on the device, so it is not
// re-applied. Needed between cells when the field is small enough that the robot
// must be carried back to the start: recalibrate while it sits still there.
bool recalibrate_otos(otos::OTOS &otos) {
	std::cout << "Keep the robot still: recalibrating OTOS IMU...\n";
	if (otos.calibrateImu(255, true) != ksfTkErrOk) {
		std::cerr << "IMU recalibration failed\n";
		return false;
	}
	if (otos.resetTracking() != ksfTkErrOk) {
		std::cerr << "resetTracking failed\n";
		return false;
	}
	std::cout << "OTOS recalibrated and re-zeroed.\n";
	return true;
}

// ---- Motors ---------------------------------------------------------------
//
// Mirrored twin-N20 drive: both M1 and M2 take the SAME signed RPM for one
// command (the shafts are mounted to spin opposite), matching the challenge
// write_motor_rpm(). A bench command is raw: no motor_rpm_command_scale, so the
// sweep measures the true command->speed relation.
class Motors {
  public:
	explicit Motors(spi::SPI &bus) : bus_(bus) {}

	bool command_rpm(std::int16_t rpm) {
		const std::int16_t clamped = std::clamp<std::int16_t>(rpm, -1500, 1500);
		const bool m1 = bus_.set_motor_speed(spi::Motor::M1, clamped);
		const bool m2 = bus_.set_motor_speed(spi::Motor::M2, clamped);
		last_rpm_ = clamped;
		return m1 && m2;
	}

	bool brake() {
		const bool m1 = bus_.set_motor_speed(spi::Motor::M1, 0);
		const bool m2 = bus_.set_motor_speed(spi::Motor::M2, 0);
		const bool br = bus_.brake();
		last_rpm_ = 0;
		return m1 && m2 && br;
	}

	// True free roll: M_DISABLE releases the closed-loop hold so the vehicle
	// coasts instead of actively resisting motion at commanded 0 RPM.
	bool coast() {
		last_rpm_ = 0;
		return bus_.disable_motors();
	}

	std::int16_t last_rpm() const { return last_rpm_; }

  private:
	spi::SPI &bus_;
	std::int16_t last_rpm_{0};
};

bool set_steering(spi::SPI &bus, const Config &config, float steer_deg) {
	const float steer_rad = steer_deg * PI / 180.0f;
	const std::uint16_t pulse =
		actuator::to_servo_pulse_us(config.servo, steer_rad);
	return bus.set_servo_pulse_us(pulse);
}

// Gate a routine that moves the wheels. [a] run, [c] recalibrate OTOS (carry the
// robot back to the start and hold still first -- for small fields), [q] abort.
// Loops on [c]. Returns true to proceed, false on [q]/interrupt. Non-TTY runs
// proceed immediately.
bool wait_to_start(const char *what, otos::OTOS &otos) {
	while (g_running) {
		std::cout << "\n" << what
				  << "\n  [a] run   [c] recalibrate OTOS (hold still)   [q] quit "
					 ": "
				  << std::flush;
		const char key = wait_for_key("acq");
		if (key == 'c') {
			recalibrate_otos(otos);
			continue;
		}
		if (key == 'a') {
			std::cout << "GO\n";
			return true;
		}
		std::cout << "\n";
		return false; // 'q' or interrupted
	}
	return false;
}

// ---- CSV ------------------------------------------------------------------

std::string make_run_dir(const std::string &mode_name) {
	const std::uint64_t stamp =
		static_cast<std::uint64_t>(std::time(nullptr));
	const std::string dir =
		"logs/measure/" + mode_name + "_" + std::to_string(stamp);
	std::error_code ec;
	std::filesystem::create_directories(dir, ec);
	if (ec) {
		std::cerr << "warning: could not create " << dir << " (" << ec.message()
				  << "); writing to current directory\n";
		return ".";
	}
	return dir;
}

void write_meta(const Config &config, const std::string &mode_name,
	float applied_scalar) {
	const std::string path = config.out_dir + "/meta.json";
	std::ofstream meta(path);
	if (!meta) {
		std::cerr << "warning: cannot write " << path << '\n';
		return;
	}
	meta << "{\n"
		 << "  \"mode\": \"" << mode_name << "\",\n"
		 << "  \"otos_linear_scalar_applied\": " << applied_scalar << ",\n"
		 << "  \"wheel_diameter_m\": " << config.wheel_diameter_m << ",\n"
		 << "  \"wheelbase_m\": " << config.wheelbase_m << ",\n"
		 << "  \"max_wheel_rpm\": " << config.max_wheel_rpm << ",\n"
		 << "  \"dwell_s\": " << config.dwell_s << ",\n"
		 << "  \"settle_s\": " << config.settle_s << ",\n"
		 << "  \"max_speed_mps\": " << config.max_speed_mps << ",\n"
		 << "  \"distance_limit_m\": " << config.distance_limit_m << ",\n"
		 << "  \"servo_center_pulse_us\": " << config.servo.servo_center_pulse_us
		 << ",\n"
		 << "  \"maximum_steering_command_deg\": "
		 << config.servo.maximum_steering_command_deg << "\n"
		 << "}\n";
	std::cout << "Wrote " << path << '\n';
}

const char *kMotionHeader =
	"t_us,combo_index,phase,cmd_wheel_rpm,servo_pulse_us,steer_deg,"
	"x_m,y_m,h_rad,vx_mps,vy_mps,speed_mps,yaw_rate_rps,ax_mps2,ay_mps2\n";

void write_motion_row(std::ofstream &csv, const OtosSample &s, int combo,
	const char *phase, std::int16_t cmd_rpm, std::uint16_t servo_pulse_us,
	float steer_deg) {
	csv << s.t_us << ',' << combo << ',' << phase << ',' << cmd_rpm << ','
		<< servo_pulse_us << ',' << steer_deg << ',' << s.x << ',' << s.y << ','
		<< s.h << ',' << s.vx << ',' << s.vy << ',' << s.speed << ','
		<< s.yaw_rate << ',' << s.ax << ',' << s.ay << '\n';
}

// ---- Modes ----------------------------------------------------------------

int run_speed_sweep(spi::SPI &bus, otos::OTOS &otos, Config &config) {
	config.out_dir = make_run_dir("speed_sweep");
	float applied_scalar = 1.0f;
	if (!setup_otos(otos, config, applied_scalar)) {
		return 1;
	}
	write_meta(config, "speed_sweep", applied_scalar);
	std::ofstream csv(config.out_dir + "/data.csv");
	if (!csv) {
		std::cerr << "cannot open data.csv\n";
		return 1;
	}
	csv << kMotionHeader;

	Motors motors(bus);
	set_steering(bus, config, 0.0f);
	motors.brake();

	const std::size_t cells =
		config.sweep_rpms.size() * config.sweep_steer_deg.size();
	std::cout << "Speed sweep: " << config.sweep_rpms.size() << " RPM x "
			  << config.sweep_steer_deg.size() << " steering = " << cells
			  << " cells, " << config.dwell_s << " s hold each.\n"
			  << "NOTE: steer=0 cells drive STRAIGHT (need a long clear lane); "
				 "steered cells drive in a circle.\n"
			  << "One cell at a time: [a] run next, [c] recalibrate OTOS (carry "
				 "back to start, hold still), [q] stop.\n"
			  << "Pose is re-zeroed on every [a], so a small field is fine.\n";

	int combo = 0;
	const std::uint64_t sample_period_us = 20'000; // ~50 Hz
	bool aborted = false;
	for (float steer_deg : config.sweep_steer_deg) {
		if (aborted) {
			break;
		}
		for (float rpm : config.sweep_rpms) {
			if (!g_running) {
				break;
			}
			const std::int16_t cmd_rpm = static_cast<std::int16_t>(
				std::clamp(rpm, -static_cast<float>(config.max_wheel_rpm),
					static_cast<float>(config.max_wheel_rpm)));

			std::ostringstream prompt;
			prompt << "cell " << combo << "/" << (cells - 1) << ": " << cmd_rpm
				   << " RPM, " << steer_deg << " deg";
			if (!wait_to_start(prompt.str().c_str(), otos)) {
				aborted = true; // [q] or interrupted
				break;
			}
			// Re-zero pose so every cell's logged trajectory starts at the
			// origin -- cheap, and keeps the per-cell logs interpretable when
			// the robot is repositioned between cells on a small field.
			otos.resetTracking();

			set_steering(bus, config, steer_deg);
			interruptible_sleep_s(0.4f); // let the servo reach the angle
			if (!motors.command_rpm(cmd_rpm)) {
				std::cerr << "motor command failed; aborting\n";
				motors.brake();
				return 1;
			}
			const std::uint16_t pulse =
				actuator::to_servo_pulse_us(config.servo,
					steer_deg * PI / 180.0f);

			// Hold: log the whole window; the analyser drops the settle prefix.
			const std::uint64_t hold_end =
				now_us() + static_cast<std::uint64_t>(config.dwell_s * 1e6f);
			bool speed_limited = false;
			while (g_running && now_us() < hold_end) {
				OtosSample s;
				if (!read_otos(otos, s)) {
					std::cerr << "OTOS read failed; aborting\n";
					motors.brake();
					return 1;
				}
				write_motion_row(
					csv, s, combo, "hold", cmd_rpm, pulse, steer_deg);
				if (s.speed > config.max_speed_mps) {
					std::cerr << "SAFETY STOP: OTOS speed " << s.speed
						      << " m/s exceeded limit " << config.max_speed_mps
						      << " m/s\n";
					speed_limited = true;
					break;
				}
				if (std::hypot(s.x, s.y) >= config.distance_limit_m) {
					std::cout << "  distance limit reached; stopping cell\n";
					break;
				}
				std::this_thread::sleep_for(
					std::chrono::microseconds(sample_period_us));
			}

			// Stop and log the decel/settle gap so coast-in is captured too.
			motors.brake();
			const std::uint16_t center_pulse =
				actuator::to_servo_pulse_us(config.servo, 0.0f);
			set_steering(bus, config, 0.0f);
			const std::uint64_t pause_end =
				now_us() + static_cast<std::uint64_t>(config.pause_s * 1e6f);
			while (g_running && now_us() < pause_end) {
				OtosSample s;
				if (read_otos(otos, s)) {
					write_motion_row(
						csv, s, combo, "stop", 0, center_pulse, 0.0f);
				}
				std::this_thread::sleep_for(
					std::chrono::microseconds(sample_period_us));
			}
			++combo;
			if (speed_limited) {
				aborted = true;
			}
		}
	}

	motors.brake();
	set_steering(bus, config, 0.0f);
	std::cout << "Speed sweep done: " << config.out_dir << "/data.csv\n";
	return 0;
}

int run_coast_down(spi::SPI &bus, otos::OTOS &otos, Config &config) {
	config.out_dir = make_run_dir("coast_down");
	float applied_scalar = 1.0f;
	if (!setup_otos(otos, config, applied_scalar)) {
		return 1;
	}
	write_meta(config, "coast_down", applied_scalar);
	std::ofstream csv(config.out_dir + "/data.csv");
	if (!csv) {
		std::cerr << "cannot open data.csv\n";
		return 1;
	}
	csv << kMotionHeader;

	Motors motors(bus);
	set_steering(bus, config, 0.0f);
	motors.brake();

	const std::int16_t cmd_rpm = static_cast<std::int16_t>(std::clamp(
		config.coast_rpm, 0.0f, static_cast<float>(config.max_wheel_rpm)));
	std::cout << "Coast-down: spin up to " << cmd_rpm
			  << " RPM straight, then release the motors and log to rest.\n"
			  << "NOTE: needs a long clear lane.\n";
	if (!wait_to_start("Coast-down will drive the robot forward", otos)) {
		motors.brake();
		return 0;
	}
	otos.resetTracking();

	const std::uint16_t center_pulse =
		actuator::to_servo_pulse_us(config.servo, 0.0f);
	const std::uint64_t sample_period_us = 10'000; // ~100 Hz for decel detail

	// Spin up and hold to steady speed.
	if (!motors.command_rpm(cmd_rpm)) {
		std::cerr << "motor command failed; aborting\n";
		motors.brake();
		return 1;
	}
	const std::uint64_t spinup_end =
		now_us() + static_cast<std::uint64_t>(config.dwell_s * 1e6f);
	while (g_running && now_us() < spinup_end) {
		OtosSample s;
		if (read_otos(otos, s)) {
			write_motion_row(csv, s, 0, "spinup", cmd_rpm, center_pulse, 0.0f);
			if (s.speed > config.max_speed_mps) {
				std::cerr << "SAFETY STOP: OTOS speed " << s.speed
				          << " m/s exceeded limit " << config.max_speed_mps
				          << " m/s\n";
				motors.brake();
				return 1;
			}
		}
		std::this_thread::sleep_for(
			std::chrono::microseconds(sample_period_us));
	}

	// Release: free roll, no brake.
	if (!motors.coast()) {
		std::cerr << "disable_motors failed; braking for safety\n";
		motors.brake();
		return 1;
	}
	std::cout << "  motors released -- coasting\n";
	const std::uint64_t coast_deadline =
		now_us() + static_cast<std::uint64_t>(config.coast_timeout_s * 1e6f);
	int below_count = 0;
	while (g_running && now_us() < coast_deadline) {
		OtosSample s;
		if (!read_otos(otos, s)) {
			break;
		}
		write_motion_row(csv, s, 0, "coast", 0, center_pulse, 0.0f);
		if (s.speed < config.coast_stop_speed_mps) {
			if (++below_count >= 10) { // ~0.1 s below threshold = stopped
				break;
			}
		} else {
			below_count = 0;
		}
		std::this_thread::sleep_for(
			std::chrono::microseconds(sample_period_us));
	}

	motors.brake();
	std::cout << "Coast-down done: " << config.out_dir << "/data.csv\n";
	return 0;
}

int run_lidar_noise(Config &config) {
	config.out_dir = make_run_dir("lidar_noise");
	std::ofstream csv(config.out_dir + "/data.csv");
	if (!csv) {
		std::cerr << "cannot open data.csv\n";
		return 1;
	}
	csv << "scan_index,t_us,points_total,rejected_quality,rejected_range,"
		   "n_segments,n_obstacles,"
		   "left_dist_m,left_rms_m,right_dist_m,right_rms_m,"
		   "front_dist_m,front_rms_m\n";
	// meta without OTOS: motors and OTOS are irrelevant here.
	{
		std::ofstream meta(config.out_dir + "/meta.json");
		meta << "{\n  \"mode\": \"lidar_noise\",\n  \"scans\": "
			 << config.scans << ",\n  \"lidar_port\": \"" << config.lidar_port
			 << "\"\n}\n";
	}

	lidar::LidarModule lidar(config.lidar_port, 1000000);
	lidar::LidarProcessor processor;
	if (!lidar.initialize() || !lidar.start()) {
		std::cerr << "LiDAR init/start failed\n";
		return 1;
	}
	std::cout << "LiDAR noise: keep the robot and surroundings STILL. "
				 "Capturing " << config.scans << " scans...\n";

	const auto emit = [](std::ofstream &out,
						  const std::optional<lidar::LineSegment> &wall) {
		if (wall.has_value()) {
			out << ',' << wall->perpendicular_distance() << ','
				<< wall->rms_error_m;
		} else {
			out << ",,"; // empty = not measured (missing-value convention)
		}
	};

	int captured = 0;
	while (g_running && captured < config.scans) {
		TimedLidarData scan;
		if (!lidar.wait_for_data(scan) || scan.points.empty()) {
			continue;
		}
		const lidar::ProcessedLidarData processed = processor.process(scan);
		csv << captured << ',' << scan.timestamp_us << ','
			<< processed.reject_stats.total << ','
			<< processed.reject_stats.rejected_quality << ','
			<< processed.reject_stats.rejected_range << ','
			<< processed.line_segments.size() << ','
			<< processed.obstacles.size();
		emit(csv, processed.walls.left);
		emit(csv, processed.walls.right);
		emit(csv, processed.walls.front);
		csv << '\n';
		++captured;
		if (captured % 50 == 0) {
			std::cout << "  " << captured << "/" << config.scans << '\n';
		}
	}
	lidar.stop();
	std::cout << "LiDAR noise done (" << captured
			  << " scans): " << config.out_dir << "/data.csv\n";
	return 0;
}

int run_lidar_capture(otos::OTOS &otos, Config &config) {
	config.out_dir = make_run_dir("lidar_capture");
	float applied_scalar = 1.0f;
	if (!setup_otos(otos, config, applied_scalar)) {
		return 1;
	}
	write_meta(config, "lidar_capture", applied_scalar);
	std::ofstream csv(config.out_dir + "/data.csv");
	if (!csv) {
		std::cerr << "cannot open data.csv\n";
		return 1;
	}
	// Long format: one row per point, tagged with its scan's OTOS motion so the
	// analyser can replicate LidarProcessor::deskew() and plot before/after.
	// forward_speed_mps (robot +Y) = vx*cos(h)+vy*sin(h); logged raw so the
	// analyser can reproduce the exact ScanMotion the production path builds.
	csv << "scan_index,scan_period_s,h_rad,vx_mps,vy_mps,speed_mps,yaw_rate_rps,"
		   "point_index,angle_deg,distance_m,scan_phase,quality\n";

	lidar::LidarModule lidar(config.lidar_port, 1000000);
	if (!lidar.initialize() || !lidar.start()) {
		std::cerr << "LiDAR init/start failed\n";
		return 1;
	}
	std::cout << "LiDAR capture: PUSH the robot smoothly by hand while "
			  << config.capture_scans << " scans are recorded.\n";
	if (!wait_to_start("Press [a], then push the robot", otos)) {
		lidar.stop();
		return 0;
	}
	otos.resetTracking();

	int captured = 0;
	std::uint64_t last_scan_us = 0;
	while (g_running && captured < config.capture_scans) {
		TimedLidarData scan;
		if (!lidar.wait_for_data(scan) || scan.points.empty()) {
			continue;
		}
		OtosSample s;
		read_otos(otos, s);
		float period_s = 0.05f;
		if (last_scan_us != 0 && scan.timestamp_us > last_scan_us) {
			period_s =
				static_cast<float>(scan.timestamp_us - last_scan_us) * 1e-6f;
		}
		last_scan_us = scan.timestamp_us;

		int point_index = 0;
		for (const auto &p : scan.points) {
			csv << captured << ',' << period_s << ',' << s.h << ',' << s.vx << ','
				<< s.vy << ',' << s.speed << ',' << s.yaw_rate << ','
				<< point_index++ << ',' << p.angle_deg << ',' << p.distance_m
				<< ',' << p.scan_phase << ',' << static_cast<int>(p.quality)
				<< '\n';
		}
		++captured;
		std::cout << "  scan " << captured << "/" << config.capture_scans
				  << " (" << scan.points.size() << " pts, speed " << s.speed
				  << " m/s)\n";
	}
	lidar.stop();
	std::cout << "LiDAR capture done: " << config.out_dir << "/data.csv\n";
	return 0;
}

void print_usage(const char *program) {
	std::cout
		<< "Usage: " << program << " <mode> [options]\n\n"
		<< "Modes (exactly one):\n"
		<< "  --speed-sweep     M-7 + M-4: RPM x steering grid, log steady "
		   "state\n"
		<< "  --coast-down      M-8: spin up, release motors, log decel\n"
		<< "  --lidar-noise     M-11: stationary scan noise floor\n"
		<< "  --lidar-capture   raw points + OTOS motion for deskew plots\n\n"
		<< "Common:\n"
		<< "  --otos-scalar V   OTOS linear correction (default 1.0956, "
		   "clamped [" << SCALAR_MIN << ".." << SCALAR_MAX << "])\n"
		<< "  --wheel-d M       wheel diameter m (default 0.0525)\n"
		<< "  --lidar-port P    default /dev/ttyAMA0\n\n"
		<< "Speed sweep:\n"
		<< "  --rpms a,b,c      raw wheel RPM cells (default 50,100,150,200; "
		   "competition uses <=300)\n"
		<< "  --steer-deg a,b   steering cells (default 0; add angles only in a wider area)\n"
		<< "  --dwell S         hold per cell (default 2.5)\n"
		<< "  --settle S        analyser-discard prefix (default 1.0)\n"
		<< "  --pause S         stopped gap per cell (default 2.0)\n\n"
		<< "  --max-speed V     OTOS hard speed stop (default 0.4 m/s)\n"
		<< "  --distance-m D    maximum travel per cell (default 2.0 m)\n\n"
		<< "Coast-down:\n"
		<< "  --coast-rpm R     spin-up RPM (default 125)\n"
		<< "  --coast-timeout S max coast log time (default 6)\n\n"
		<< "LiDAR:\n"
		<< "  --scans N         lidar-noise scan count (default 200)\n"
		<< "  --capture-scans N lidar-capture scan count (default 20)\n";
}

std::optional<float> next_float(int &i, int argc, char **argv) {
	if (i + 1 >= argc) {
		return std::nullopt;
	}
	try {
		return std::stof(argv[++i]);
	} catch (const std::exception &) {
		return std::nullopt;
	}
}

} // namespace

int main(int argc, char **argv) {
	Config config;

	for (int i = 1; i < argc; ++i) {
		const std::string opt(argv[i]);
		auto need_float = [&](float &dst) {
			if (auto v = next_float(i, argc, argv)) {
				dst = *v;
			} else {
				std::cerr << opt << " needs a number\n";
				std::exit(2);
			}
		};

		if (opt == "--help" || opt == "-h") {
			print_usage(argv[0]);
			return 0;
		} else if (opt == "--speed-sweep") {
			config.mode = Mode::SPEED_SWEEP;
		} else if (opt == "--coast-down") {
			config.mode = Mode::COAST_DOWN;
		} else if (opt == "--lidar-noise") {
			config.mode = Mode::LIDAR_NOISE;
		} else if (opt == "--lidar-capture") {
			config.mode = Mode::LIDAR_CAPTURE;
		} else if (opt == "--otos-scalar") {
			need_float(config.otos_scalar);
		} else if (opt == "--wheel-d") {
			need_float(config.wheel_diameter_m);
		} else if (opt == "--lidar-port") {
			if (i + 1 < argc) {
				config.lidar_port = argv[++i];
			}
		} else if (opt == "--rpms") {
			if (i + 1 < argc) {
				config.sweep_rpms = parse_float_list(argv[++i]);
			}
		} else if (opt == "--steer-deg") {
			if (i + 1 < argc) {
				config.sweep_steer_deg = parse_float_list(argv[++i]);
			}
		} else if (opt == "--dwell") {
			need_float(config.dwell_s);
		} else if (opt == "--settle") {
			need_float(config.settle_s);
		} else if (opt == "--pause") {
			need_float(config.pause_s);
		} else if (opt == "--max-speed") {
			need_float(config.max_speed_mps);
		} else if (opt == "--distance-m") {
			need_float(config.distance_limit_m);
		} else if (opt == "--coast-rpm") {
			need_float(config.coast_rpm);
		} else if (opt == "--coast-timeout") {
			need_float(config.coast_timeout_s);
		} else if (opt == "--scans") {
			float v = 0;
			need_float(v);
			config.scans = static_cast<int>(v);
		} else if (opt == "--capture-scans") {
			float v = 0;
			need_float(v);
			config.capture_scans = static_cast<int>(v);
		} else {
			std::cerr << "Unknown option: " << opt << "\n\n";
			print_usage(argv[0]);
			return 2;
		}
	}

	if (config.mode == Mode::NONE) {
		std::cerr << "Choose exactly one mode.\n\n";
		print_usage(argv[0]);
		return 2;
	}
	if (config.sweep_rpms.empty() || config.sweep_steer_deg.empty()) {
		std::cerr << "empty --rpms or --steer-deg list\n";
		return 2;
	}
	if (config.max_speed_mps <= 0.0f || config.distance_limit_m <= 0.0f) {
		std::cerr << "--max-speed and --distance-m must be positive\n";
		return 2;
	}

	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);

	// Raw terminal for single-key [a]/[q] gating; restored on scope exit.
	TerminalRawMode terminal;

	// LiDAR-only modes never touch SPI/OTOS motion.
	if (config.mode == Mode::LIDAR_NOISE) {
		return run_lidar_noise(config);
	}

	otos::OTOS otos;
	if (config.mode == Mode::LIDAR_CAPTURE) {
		return run_lidar_capture(otos, config);
	}

	// Motion modes: bring up SPI, guarantee a stop on every exit path.
	spi::SPI bus;
	if (!bus.initialize()) {
		std::cerr << "SPI initialize failed\n";
		return 1;
	}
	Motors safety(bus);
	safety.brake();
	set_steering(bus, config, 0.0f);

	int rc = 1;
	if (config.mode == Mode::SPEED_SWEEP) {
		rc = run_speed_sweep(bus, otos, config);
	} else if (config.mode == Mode::COAST_DOWN) {
		rc = run_coast_down(bus, otos, config);
	}

	safety.brake();
	set_steering(bus, config, 0.0f);
	bus.close();
	if (!g_running) {
		std::cout << "\nInterrupted -- motors braked, servo centred.\n";
	}
	return rc;
}
