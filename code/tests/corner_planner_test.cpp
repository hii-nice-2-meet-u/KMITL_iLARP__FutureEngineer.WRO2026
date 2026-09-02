#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "corner_planner.hpp"
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

navigation::CornerPlan plan(float cx, float cy, float px, float py, float ph,
	float offset, float interior_sign) {
	const kinematics::BicycleModel model; // wheelbase 0.16375, gain 1.0
	const float max_steering_rad = 38.0f * PI / 180.0f;
	return navigation::CornerPlanner::plan(
		cx, cy, px, py, ph, offset, interior_sign, model, max_steering_rad);
}

} // namespace

int main() {
	// --- world->robot transform at heading 0: world +x -> right, +y -> forward.
	{
		const auto p = plan(0.10f, 0.50f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
		expect(p.valid, "transform: valid ahead");
		expect_near(p.apex_forward_m, 0.50f, 1e-4f, "transform forward = dy");
		expect_near(p.apex_lateral_m, 0.10f, 1e-4f, "transform lateral = dx");
	}

	// --- pure-pursuit curvature law: kappa = 2*lat/(lat^2+fwd^2), zero offset.
	{
		const auto p = plan(0.30f, 0.40f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
		const float expected = 2.0f * 0.30f / (0.30f * 0.30f + 0.40f * 0.40f);
		expect_near(p.curvature_1pm, expected, 1e-4f, "pursuit curvature law");
		expect(p.curvature_1pm > 0.0f, "target on the right curves right (kappa>0)");
	}

	// --- straight-ahead target -> ~zero curvature.
	{
		const auto p = plan(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
		expect_near(p.curvature_1pm, 0.0f, 1e-4f, "straight ahead -> kappa 0");
	}

	// --- CW corner (interior_sign +1): apex shifts left of the corner point.
	{
		const auto p = plan(0.50f, 0.50f, 0.0f, 0.0f, 0.0f, 0.20f, 1.0f);
		expect_near(p.apex_lateral_m, 0.30f, 1e-4f, "CW apex_lat = lat - offset");
		expect(p.curvature_1pm > 0.0f, "CW corner still curves right here");
	}

	// --- CCW corner (interior_sign -1): mirror, curves left.
	{
		const auto p = plan(-0.50f, 0.50f, 0.0f, 0.0f, 0.0f, 0.20f, -1.0f);
		expect_near(p.apex_lateral_m, -0.30f, 1e-4f, "CCW apex_lat = lat + offset");
		expect(p.curvature_1pm < 0.0f, "CCW corner curves left (kappa<0)");
	}

	// --- apex behind the vehicle -> invalid (caller falls back to legacy arc).
	{
		const auto p = plan(0.0f, -0.50f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
		expect(!p.valid, "apex behind -> invalid");
	}

	// --- curvature is clamped to the steering-limited maximum.
	{
		const kinematics::BicycleModel model;
		const float max_curv = model.max_curvature(38.0f * PI / 180.0f);
		const auto p = plan(0.30f, 0.03f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
		expect(std::abs(p.curvature_1pm) <= max_curv + 1e-4f,
			"curvature clamped to max_curvature");
	}

	// --- pose heading is respected: same world corner, robot rotated by +90 deg.
	// At heading +pi/2: forward = -dx, lateral = dy.
	{
		const auto p = plan(0.0f, 0.40f, 0.0f, 0.0f, PI / 2.0f, 0.0f, 1.0f);
		expect_near(p.apex_forward_m, 0.0f, 1e-4f, "rotated forward = -dx");
		expect_near(p.apex_lateral_m, 0.40f, 1e-4f, "rotated lateral = dy");
	}

	std::cout << "PASS: corner planner (transform, pursuit law, CW/CCW, clamp)\n";
	return 0;
}
