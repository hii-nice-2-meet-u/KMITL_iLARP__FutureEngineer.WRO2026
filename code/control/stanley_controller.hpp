#pragma once

#include <algorithm>
#include <cmath>

#include "pid.hpp"

namespace control {

struct StanleyConfig {
	// Cross-track correction gain
	float k{1.0f};

	// Prevent aggressive steering when speed is near zero
	float softening_speed_mps{0.20f};

	// Physical steering limit of Ackermann front wheels
	float max_steering_rad{0.785398f}; // 30 deg

	PIDConfig heading_pid{
		1.00f, 0.12f, 0.025f, -0.785398f, 0.785398f, -0.50f, 0.50f, 0.10f};
};

class StanleyController {
  public:
	explicit StanleyController(StanleyConfig config = {});

	/**
	 * Stanley steering controller
	 *
	 * delta =
	 *     heading_error
	 *     + atan2(k * cross_track_error,
	 *             speed + softening_speed)
	 *
	 * @param cross_track_error_m
	 *     Lateral error from target path [m]
	 *
	 * @param heading_error_rad
	 *     Path heading - vehicle heading [rad]
	 *
	 * @param speed_mps
	 *     Current forward vehicle speed [m/s]
	 *
	 * @return
	 *     Desired front-wheel steering angle [rad]
	 *     negative = LEFT
	 *     positive = RIGHT
	 */
	float calculate(float cross_track_error_m, float heading_error_rad,
		float speed_mps, float dt_s) const;

	void reset();

	void set_config(const StanleyConfig &config);

	const StanleyConfig &config() const { return config_; }

  private:
	StanleyConfig config_;
	mutable PID heading_pid_;
};

} // namespace control
