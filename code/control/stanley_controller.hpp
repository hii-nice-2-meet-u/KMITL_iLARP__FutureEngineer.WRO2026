#include "stanley_controller.hpp"


namespace control {

StanleyController::StanleyController(StanleyConfig config) : config_(config) {}

float StanleyController::calculate(
	float cross_track_error_m, float heading_error_rad, float speed_mps) const {

	const float speed = std::max(std::abs(speed_mps), 0.0f);

	const float cross_track_term = std::atan2(
		config_.k * cross_track_error_m, speed + config_.softening_speed_mps);

	const float steering = heading_error_rad + cross_track_term;

	return std::clamp(
		steering, -config_.max_steering_rad, config_.max_steering_rad);
}

void StanleyController::set_config(const StanleyConfig &config) {

	config_ = config;
}

} // namespace control