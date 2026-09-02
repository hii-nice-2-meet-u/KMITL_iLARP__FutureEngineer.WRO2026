#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "kinematics.hpp"

namespace {

constexpr float PI = 3.14159265358979323846f;

[[noreturn]] void fail(const std::string &message) {
	std::cerr << "FAIL: " << message << '\n';
	std::exit(1);
}

void expect(bool condition, const std::string &message) {
	if (!condition) {
		fail(message);
	}
}

void expect_near(float actual, float expected, float tolerance,
	const std::string &message) {
	if (std::abs(actual - expected) > tolerance) {
		fail(message + " (expected " + std::to_string(expected) + ", got " +
			std::to_string(actual) + ")");
	}
}

} // namespace

int main() {
	const kinematics::BicycleModel model; // wheelbase 0.16375, gain 1.0

	// --- reference values (MOTION_MODEL_IMPLEMENTATION_PLAN.md section 4.1) ---
	// delta -> kappa -> radius, at the default geometry.
	struct Ref {
		float deg;
		float kappa;
		float radius;
	};
	const Ref refs[] = {
		{20.0f, 2.2228f, 0.450f},
		{38.0f, 4.7712f, 0.210f},
		{45.0f, 6.1069f, 0.164f},
	};
	for (const Ref &r : refs) {
		const float kappa = model.curvature_for_steering(r.deg * PI / 180.0f);
		expect_near(kappa, r.kappa, 1e-3f,
			"kappa at " + std::to_string(r.deg) + " deg");
		expect_near(1.0f / kappa, r.radius, 1e-3f,
			"radius at " + std::to_string(r.deg) + " deg");
	}

	// --- round-trip: steering_for_curvature is the inverse over [-45, 45] ---
	for (float deg = -45.0f; deg <= 45.0f; deg += 1.0f) {
		const float rad = deg * PI / 180.0f;
		const float round_trip =
			model.steering_for_curvature(model.curvature_for_steering(rad));
		expect_near(round_trip, rad, 1e-6f,
			"round trip at " + std::to_string(deg) + " deg");
	}

	// --- both conversions are odd and strictly monotonic ---
	expect_near(model.curvature_for_steering(0.0f), 0.0f, 1e-9f,
		"kappa(0) is 0");
	expect_near(model.steering_for_curvature(0.0f), 0.0f, 1e-9f,
		"delta(0) is 0");
	float previous_kappa = model.curvature_for_steering(-0.78f);
	float previous_delta = model.steering_for_curvature(-8.0f);
	for (float s = -0.77f; s <= 0.78f; s += 0.01f) {
		const float kappa = model.curvature_for_steering(s);
		expect(kappa > previous_kappa, "curvature_for_steering monotonic");
		previous_kappa = kappa;
	}
	for (float k = -7.9f; k <= 8.0f; k += 0.1f) {
		const float delta = model.steering_for_curvature(k);
		expect(delta > previous_delta, "steering_for_curvature monotonic");
		previous_delta = delta;
	}
	expect_near(model.curvature_for_steering(-0.3f),
		-model.curvature_for_steering(0.3f), 1e-6f, "kappa is odd");
	expect_near(model.steering_for_curvature(-2.0f),
		-model.steering_for_curvature(2.0f), 1e-6f, "delta is odd");

	// --- max_curvature matches curvature at the clamp ---
	expect_near(model.max_curvature(38.0f * PI / 180.0f), 4.7712f, 1e-3f,
		"max_curvature at 38 deg");

	// --- curvature rate is positive, finite, and grows with |kappa| ---
	const float rate_straight = model.max_curvature_rate(0.0f, 3.0f);
	const float rate_curved = model.max_curvature_rate(4.0f, 3.0f);
	expect(std::isfinite(rate_straight) && rate_straight > 0.0f,
		"rate at kappa=0 positive/finite");
	expect(rate_curved > rate_straight,
		"curvature rate grows with |kappa|");
	// symmetric in kappa sign
	expect_near(model.max_curvature_rate(-4.0f, 3.0f), rate_curved, 1e-4f,
		"curvature rate symmetric in sign");
	// zero servo rate -> zero curvature rate
	expect_near(model.max_curvature_rate(2.0f, 0.0f), 0.0f, 1e-9f,
		"zero servo rate gives zero curvature rate");

	// --- speed-for-lateral-limit ---
	// straight path is capped by the ceiling, not sent to infinity
	expect_near(
		kinematics::BicycleModel::speed_for_lateral_limit(0.0f, 0.5f, 0.45f),
		0.45f, 1e-6f, "straight path returns the speed ceiling");
	// v = sqrt(a / kappa), below the ceiling
	expect_near(kinematics::BicycleModel::speed_for_lateral_limit(
					2.2228f, 1.4f, 2.0f),
		std::sqrt(1.4f / 2.2228f), 1e-4f, "lateral-limited speed at R=0.45");
	// binds to the ceiling when sqrt(a/kappa) exceeds it
	expect_near(kinematics::BicycleModel::speed_for_lateral_limit(
					0.5f, 1.4f, 0.30f),
		0.30f, 1e-6f, "speed limit binds to the mode ceiling");

	// --- curvature_gain scales achieved curvature for the same steering ---
	kinematics::BicycleModel scrub;
	scrub.curvature_gain = 1.35f;
	expect_near(scrub.curvature_for_steering(0.3f),
		1.35f * model.curvature_for_steering(0.3f), 1e-6f,
		"gain scales curvature");
	// and the inverse still round-trips under a non-unit gain
	expect_near(scrub.steering_for_curvature(scrub.curvature_for_steering(0.3f)),
		0.3f, 1e-6f, "round trip holds under gain != 1");

	std::cout << "PASS: bicycle-model kinematics (conversions, rate, speed cap)\n";
	return 0;
}
