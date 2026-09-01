#pragma once

#include <chrono>

namespace control {

struct PIDConfig {
	float kp{0.0f};
	float ki{0.0f};
	float kd{0.0f};

	float min_output{-1.0f};
	float max_output{1.0f};

	float min_integral{-1.0f};
	float max_integral{1.0f};

	float max_dt_s{0.10f};
};

class PID {
  public:
	explicit PID(PIDConfig config = {});

	float calculate(float setpoint, float current);
	float calculate(float setpoint, float current, float dt_s);

	void reset();

	void set_config(const PIDConfig &config);

	const PIDConfig &config() const { return config_; }

	float error() const { return previous_error_; }

	// Telemetry accessors. They expose internal state so a caller can log the
	// controller's own contribution instead of only the summed command; tuning
	// kp/ki/kd from a combined output alone is guesswork.
	float integral() const { return integral_; }

	float last_output() const { return last_output_; }

  private:
	using Clock = std::chrono::steady_clock;

	PIDConfig config_;

	float integral_{0.0f};
	float previous_error_{0.0f};
	float last_output_{0.0f};

	Clock::time_point previous_time_{};

	bool initialized_{false};
};

} // namespace control