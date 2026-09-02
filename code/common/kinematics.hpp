#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace kinematics {

struct ClothoidReference {
	float total_angle_rad{0.0f};
	float radius_m{0.4f};
	float ramp_length_m{0.10f};

	float total_length_m() const {
		return std::max(0.0f, radius_m) * std::abs(total_angle_rad);
	}
	float effective_ramp_m() const {
		return std::min(total_length_m() * 0.5f,
			std::max(1e-4f, ramp_length_m));
	}
	float peak_curvature_1pm() const {
		const float length = total_length_m();
		const float ramp = effective_ramp_m();
		return length > 1e-6f ? std::abs(total_angle_rad) /
			(std::max(0.0f, length - 2.0f * ramp) + ramp) : 0.0f;
	}
	float curvature_at_distance(float distance_m) const {
		const float s = std::clamp(distance_m, 0.0f, total_length_m());
		const float ramp = effective_ramp_m();
		const float plateau_end = total_length_m() - ramp;
		const float peak = peak_curvature_1pm();
		if (s < ramp) return peak * s / ramp;
		if (s <= plateau_end) return peak;
		return peak * (total_length_m() - s) / ramp;
	}
	float heading_progress_at_distance(float distance_m) const {
		const float s = std::clamp(distance_m, 0.0f, total_length_m());
		const float ramp = effective_ramp_m();
		const float plateau_end = total_length_m() - ramp;
		const float peak = peak_curvature_1pm();
		float integral = 0.0f;
		if (s <= ramp) integral = peak * s * s / (2.0f * ramp);
		else if (s <= plateau_end) integral = peak * (s - ramp * 0.5f);
		else integral = peak * (plateau_end - ramp * 0.5f) +
			peak * (s - plateau_end) *
			(1.0f - (s - plateau_end) / (2.0f * ramp));
		return integral;
	}
};

// The bicycle model, in one place.
//
// Curvature is the control stack's currency because it superposes linearly:
// a wall-following curvature and an obstacle-avoidance curvature may be added,
// whereas two steering angles may not. delta = atan(L * kappa) is nonlinear --
// the error from adding angles instead of curvatures is 4.3% at 20 deg and
// grows from there.
//
// Sign convention matches the steering convention throughout the codebase:
//   kappa < 0  curves LEFT
//   kappa > 0  curves RIGHT
//
// Nothing consumes this yet: it is introduced ahead of the migration that
// moves the controllers onto curvature, so it can be reviewed and unit-tested
// in isolation first.
struct BicycleModel {
	// Rear-axle centre to front-axle centre. A measurement, never a gain.
	float wheelbase_m{0.16375f};

	// Ratio of the curvature the vehicle actually achieves to the curvature the
	// ideal Ackermann model predicts for the same steering angle.
	//
	// 1.0 is the textbook model. This chassis measured a minimum radius of
	// 0.155 m where the geometry allows only 0.210 m at the 38 deg clamp, which
	// implies a gain near 1.35 if the clamp was active at that moment (see
	// VEHICLE_MECHANICS_REVIEW.md section 6). The leading explanation is that
	// write_motor_rpm() drives both rear motors at the same RPM, so the axle
	// cannot roll freely through a turn and scrubs, adding yaw beyond Ackermann.
	//
	// MUST stay at 1.0 until measured by the M-4 procedure. Setting it from the
	// log instead of from a floor measurement would fold OTOS angular-scale
	// error into the vehicle model.
	float curvature_gain{1.0f};

	float curvature_for_steering(float steering_rad) const {
		const float wheelbase = std::max(1e-4f, wheelbase_m);
		return curvature_gain * std::tan(steering_rad) / wheelbase;
	}

	float steering_for_curvature(float curvature_1pm) const {
		const float wheelbase = std::max(1e-4f, wheelbase_m);
		const float gain = std::max(1e-3f, curvature_gain);
		return std::atan(wheelbase * curvature_1pm / gain);
	}

	float max_curvature(float max_steering_rad) const {
		return curvature_for_steering(std::abs(max_steering_rad));
	}

	// Largest |d(kappa)/dt| the servo can deliver at the current curvature.
	//
	// Differentiating delta = atan(L * kappa / g):
	//     d(delta)/d(kappa) = (L / g) / (1 + (L * kappa / g)^2)
	// so a fixed servo rate maps to a curvature rate that GROWS with curvature.
	// A single constant kappa-rate limit would therefore be correct only at
	// kappa = 0 and needlessly slow everywhere else.
	float max_curvature_rate(
		float curvature_1pm, float max_steering_rate_rad_s) const {
		const float wheelbase = std::max(1e-4f, wheelbase_m);
		const float gain = std::max(1e-3f, curvature_gain);
		const float scaled = wheelbase * curvature_1pm / gain;
		return std::max(0.0f, max_steering_rate_rad_s) *
			(1.0f + scaled * scaled) * gain / wheelbase;
	}

	// Convert the actuator's per-tick pulse step into a curvature step. The
	// asymmetric pulse spans are intentional: the real servo has different
	// travel on either side of centre. All pulse<->steering kinematics remain in
	// this header so the navigation conditioner does not duplicate atan/tan.
	float max_curvature_step_from_pulse(
		float curvature_1pm, float target_curvature_1pm,
		std::uint16_t maximum_pulse_step_us,
		std::uint16_t servo_min_pulse_us, std::uint16_t servo_center_pulse_us,
		std::uint16_t servo_max_pulse_us,
		float maximum_steering_command_deg) const {
		const float max_deg = std::max(0.1f,
			std::abs(maximum_steering_command_deg));
		const float min_pulse = static_cast<float>(
			std::min(servo_min_pulse_us, servo_max_pulse_us));
		const float max_pulse = static_cast<float>(
			std::max(servo_min_pulse_us, servo_max_pulse_us));
		const float centre = std::clamp(static_cast<float>(
			servo_center_pulse_us), min_pulse, max_pulse);
		const float steering = steering_for_curvature(curvature_1pm);
		const float command_deg = std::clamp(
			steering * 180.0f / 3.14159265358979323846f, -max_deg, max_deg);
		const float span = command_deg >= 0.0f
			? max_pulse - centre : centre - min_pulse;
		const float pulse = centre + command_deg / max_deg * span;
		const float direction = target_curvature_1pm >= curvature_1pm ? 1.0f : -1.0f;
		const float target_pulse = std::clamp(pulse + direction *
			static_cast<float>(maximum_pulse_step_us), min_pulse, max_pulse);
		const float target_command_deg = span > 1e-6f
			? (target_pulse - centre) / span * max_deg : command_deg;
		const float target_curvature = curvature_for_steering(
			target_command_deg * 3.14159265358979323846f / 180.0f);
		return std::max(1e-6f, std::abs(target_curvature - curvature_1pm));
	}

	// Speed at which a given curvature produces the given lateral acceleration.
	// Guarded so a near-straight path does not return an unbounded speed.
	static float speed_for_lateral_limit(float curvature_1pm,
		float max_lateral_acceleration_mps2, float maximum_speed_mps) {
		constexpr float MINIMUM_CURVATURE_1PM = 1e-3f;
		const float curvature =
			std::max(MINIMUM_CURVATURE_1PM, std::abs(curvature_1pm));
		const float limit =
			std::sqrt(std::max(0.0f, max_lateral_acceleration_mps2) / curvature);
		return std::min(limit, maximum_speed_mps);
	}
};

} // namespace kinematics
