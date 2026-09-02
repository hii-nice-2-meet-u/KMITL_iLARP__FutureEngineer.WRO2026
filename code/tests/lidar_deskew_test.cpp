// Unit tests for LidarProcessor::deskew (P-21). The transform is exercised in
// isolation; nothing enables it on the robot yet (ScanMotion.valid stays false
// until the M-1 hardware check).

#include "lidar_processor.hpp"

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

} // namespace

int main() {
	const lidar::LidarProcessor processor;
	const lidar::CartesianPoint p{0.30f, 1.00f, 0.0f};

	// 1. Δt = 0 (sample at scan end, phase = 1) -> identity, for any motion.
	{
		lidar::ScanMotion m{0.42f, 1.0f, 0.05f, true};
		const auto out = processor.deskew(p, 1.0f, m);
		expect_near(out.x_m, p.x_m, 1e-6f, "phase=1 leaves x");
		expect_near(out.y_m, p.y_m, 1e-6f, "phase=1 leaves y");
	}

	// 2. Zero twist -> identity, for any phase.
	{
		lidar::ScanMotion m{0.0f, 0.0f, 0.05f, true};
		const auto out = processor.deskew(p, 0.0f, m);
		expect_near(out.x_m, p.x_m, 1e-6f, "zero twist leaves x");
		expect_near(out.y_m, p.y_m, 1e-6f, "zero twist leaves y");
	}

	// 3. Pure forward motion, no rotation: the point measured a full period
	//    earlier (phase=0) must move back by v*period along +Y (the body
	//    advanced +Y, so the older point is further ahead in the end frame...
	//    p_end = p - v*dt, dt = period).
	{
		lidar::ScanMotion m{0.40f, 0.0f, 0.05f, true};
		const auto out = processor.deskew(p, 0.0f, m);
		expect_near(out.x_m, 0.30f, 1e-6f, "pure forward leaves x");
		expect_near(out.y_m, 1.00f - 0.40f * 0.05f, 1e-6f,
			"pure forward shifts y by v*dt");
	}

	// 4. Continuity across the w->0 branch: the exact form at a tiny w must
	//    agree with the straight-line limit to high precision.
	{
		lidar::ScanMotion big{0.40f, 9.0e-4f, 0.05f, true}; // just under 1e-3
		lidar::ScanMotion lim{0.40f, 0.0f, 0.05f, true};
		const auto a = processor.deskew(p, 0.0f, big);
		const auto b = processor.deskew(p, 0.0f, lim);
		expect_near(a.x_m, b.x_m, 5e-5f, "w->0 continuity x");
		expect_near(a.y_m, b.y_m, 5e-5f, "w->0 continuity y");
	}

	// 5. Exact-vs-straight-line error is bounded by 0.5*|w|*|v|*dt^2. Compare
	//    the exact transform against a hand-rolled straight-line-only deskew at
	//    the representative operating point.
	{
		const float v = 0.42f, w = 1.0f, period = 0.05f, phase = 0.0f;
		const float dt = (1.0f - phase) * period;
		lidar::ScanMotion m{v, w, period, true};
		const auto exact = processor.deskew(p, phase, m);

		// Straight-line reference: translate by v*dt along +Y, rotate by -w*dt.
		const float px = p.x_m, py = p.y_m - v * dt;
		const float th = -w * dt;
		const float rx = std::cos(th) * px - std::sin(th) * py;
		const float ry = std::sin(th) * px + std::cos(th) * py;

		const float diff = std::hypot(exact.x_m - rx, exact.y_m - ry);
		const float bound = 0.5f * std::abs(w) * std::abs(v) * dt * dt;
		expect(diff <= bound + 1e-6f,
			"exact-vs-straightline within 0.5*|w|*|v|*dt^2");
	}

	// 6. Rotation sign: expressing an earlier sample in the scan-end frame
	//    applies R(-w*dt). With positive yaw (CCW), theta = -w*dt < 0, so a
	//    point dead ahead (+Y) rotates toward +X in the end frame.
	{
		lidar::ScanMotion m{0.0f, 1.0f, 0.05f, true};
		const lidar::CartesianPoint ahead{0.0f, 1.0f, 1.0f};
		const auto out = processor.deskew(ahead, 0.0f, m);
		expect(out.x_m > 0.0f, "CCW yaw rotates a +Y point toward +X");
		expect_near(std::hypot(out.x_m, out.y_m), 1.0f, 1e-5f,
			"rotation preserves range");
	}

	if (failures == 0) {
		std::cout << "PASS: LidarProcessor deskew\n";
		return 0;
	}
	std::cerr << failures << " deskew assertion(s) failed\n";
	return 1;
}
