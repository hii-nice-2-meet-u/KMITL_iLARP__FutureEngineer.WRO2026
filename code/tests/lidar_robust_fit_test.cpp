#include "lidar_processor.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {
int failures = 0;

void expect(bool condition, const std::string &message) {
	if (!condition) {
		std::cerr << "FAIL: " << message << '\n';
		++failures;
	}
}
} // namespace

int main() {
	lidar::LidarProcessor processor;
	std::vector<lidar::CartesianPoint> points;
	for (int i = -10; i <= 10; ++i) {
		const float x = 0.04f * static_cast<float>(i);
		points.push_back({x, 0.50f, 0.50f});
	}
	// A gross return that would move ordinary TLS by centimetres.
	points.push_back({0.0f, 1.50f, 1.50f});

	const auto fit = processor.fit_line_segment(points);
	expect(fit.has_value(), "robust fit returns a line");
	if (fit.has_value()) {
		expect(std::abs(fit->perpendicular_distance() - 0.50f) < 0.002f,
			"gross outlier does not move wall distance");
		expect(fit->length() > 0.70f, "fit preserves the wall extent");
		expect(fit->rms_error_m < 0.01f,
			"Huber error remains small despite gross outlier");
	}

	if (failures == 0) {
		std::cout << "PASS: LidarProcessor robust fit\n";
		return 0;
	}
	return 1;
}
