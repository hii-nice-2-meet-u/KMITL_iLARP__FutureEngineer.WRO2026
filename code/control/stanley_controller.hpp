#pragma once

#include <algorithm>
#include <cmath>

namespace control {

struct StanleyConfig {
	// Cross-track correction gain
	float k{1.0f};

	// Prevent aggressive steering when speed is near zero
	float softening_speed_mps{0.20f};

	// Physical steering limit of Ackermann front wheels
	float max_steering_rad{0.523599f}; // 30 deg
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
		float speed_mps) const;

	void set_config(const StanleyConfig &config);

	const StanleyConfig &config() const { return config_; }

  private:
	StanleyConfig config_;
};

} // namespace control