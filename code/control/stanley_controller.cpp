#include "stanley_controller.hpp"

namespace control {

StanleyController::StanleyController(StanleyConfig config)
	: config_(config), heading_pid_(config.heading_pid) {}

float StanleyController::calculate(float cross_track_error_m,
	float heading_error_rad, float speed_mps, float dt_s) const {

	const float speed = std::max(std::abs(speed_mps), 0.0f);

	const float cross_track_term = std::atan2(
		config_.k * cross_track_error_m, speed + config_.softening_speed_mps);

	const float heading_term =
		heading_pid_.calculate(0.0f, -heading_error_rad, dt_s);

	const float steering = heading_term + cross_track_term;

	last_cross_track_term_rad_ = cross_track_term;
	last_heading_term_rad_ = heading_term;

	return std::clamp(
		steering, -config_.max_steering_rad, config_.max_steering_rad);
}

float StanleyController::calculate_curvature(float cross_track_error_m,
	float heading_error_rad, float speed_mps, float dt_s,
	const kinematics::BicycleModel &model) const {
	return model.curvature_for_steering(calculate(
		cross_track_error_m, heading_error_rad, speed_mps, dt_s));
}

void StanleyController::set_config(const StanleyConfig &config) {

	config_ = config;
	heading_pid_.set_config(config.heading_pid);
}

void StanleyController::reset() {
	heading_pid_.reset();
	last_cross_track_term_rad_ = 0.0f;
	last_heading_term_rad_ = 0.0f;
}

} // namespace control
