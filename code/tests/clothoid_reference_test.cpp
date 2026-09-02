// Property tests for kinematics::ClothoidReference (P-10). The corner reference
// ramps curvature linearly in from 0, holds a plateau, and ramps back to 0; the
// defining invariant is that its heading integrates to exactly the requested
// corner angle. Nothing tested this until now.

#include "kinematics.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

int failures = 0;

void expect(bool ok, const std::string &m) {
	if (!ok) {
		std::cerr << "FAIL: " << m << '\n';
		++failures;
	}
}

void expect_near(float got, float want, float tol, const std::string &m) {
	if (std::abs(got - want) > tol) {
		std::cerr << "FAIL: " << m << " -- got " << got << " want " << want
				  << " (tol " << tol << ")\n";
		++failures;
	}
}

constexpr float PI = 3.14159265358979323846f;

void check_corner(float angle_rad, float radius_m, float ramp_m) {
	const kinematics::ClothoidReference c{angle_rad, radius_m, ramp_m};
	const float L = c.total_length_m();
	const std::string tag = "(" + std::to_string(angle_rad) + "," +
		std::to_string(radius_m) + "," + std::to_string(ramp_m) + ") ";

	// Defining invariant: total heading == requested angle.
	expect_near(c.heading_progress_at_distance(L), std::abs(angle_rad), 1e-4f,
		tag + "heading integrates to total angle");

	// Curvature is zero at both ends and peaks in the middle.
	expect_near(c.curvature_at_distance(0.0f), 0.0f, 1e-5f, tag + "kappa(0)=0");
	expect_near(c.curvature_at_distance(L), 0.0f, 1e-5f, tag + "kappa(L)=0");
	expect_near(c.curvature_at_distance(L * 0.5f), c.peak_curvature_1pm(), 1e-5f,
		tag + "mid = peak");

	// Peak curvature is at least the nominal 1/R (it is compressed into
	// L - ramp, so it must exceed the plateau-only value).
	expect(c.peak_curvature_1pm() >= 1.0f / radius_m - 1e-4f,
		tag + "peak >= 1/R");

	// Heading is monotone non-decreasing along the arc.
	float previous = -1.0f;
	bool monotone = true;
	for (int i = 0; i <= 20; ++i) {
		const float h = c.heading_progress_at_distance(L * i / 20.0f);
		if (h + 1e-5f < previous) {
			monotone = false;
		}
		previous = h;
	}
	expect(monotone, tag + "heading is monotone");
}

} // namespace

int main() {
	// A normal WRO 90-degree corner at the configured geometry.
	check_corner(PI * 0.5f, 0.45f, 0.10f);
	// A gentler, larger corner.
	check_corner(PI * 0.5f, 0.80f, 0.15f);
	// A degenerate short corner where the ramp would exceed half the arc:
	// effective_ramp clamps and the heading invariant must still hold.
	check_corner(PI * 0.5f, 0.12f, 0.30f);

	if (failures == 0) {
		std::cout << "PASS: ClothoidReference\n";
		return 0;
	}
	std::cerr << failures << " clothoid assertion(s) failed\n";
	return 1;
}
