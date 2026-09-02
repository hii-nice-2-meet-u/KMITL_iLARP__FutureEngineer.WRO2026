// Golden-value tests pinning the servo-pulse arithmetic in
// common/servo_pulse.hpp. These values were computed by hand from the mapping
// that previously lived inside ActuatorOutput; if any of them changes, the
// steering-rad -> pulse behaviour of the real robot changed with it.

#include "servo_pulse.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

namespace {

int failures = 0;

void expect_pulse(std::uint16_t got, std::uint16_t want, const std::string &m) {
	if (got != want) {
		std::cerr << "FAIL: " << m << " -- got " << got << " want " << want
				  << '\n';
		++failures;
	}
}

constexpr float PI = 3.14159265358979323846f;

} // namespace

int main() {
	// The default configuration matches ActuatorConfig's defaults: 950/1475/2000
	// pulse, 500 us step, sign +1, 45 deg command range.
	const actuator::ServoPulseConfig cfg{};

	// --- to_servo_pulse_us: anchor points -----------------------------------
	// Zero steering -> centre.
	expect_pulse(actuator::to_servo_pulse_us(cfg, 0.0f), 1475, "zero -> centre");

	// Full right (+45 deg) -> max; full left (-45 deg) -> min.
	expect_pulse(actuator::to_servo_pulse_us(cfg, 45.0f * PI / 180.0f), 2000,
		"+45 deg -> max");
	expect_pulse(actuator::to_servo_pulse_us(cfg, -45.0f * PI / 180.0f), 950,
		"-45 deg -> min");

	// Beyond the command range clamps to the extremes.
	expect_pulse(actuator::to_servo_pulse_us(cfg, 90.0f * PI / 180.0f), 2000,
		"+90 deg clamps to max");
	expect_pulse(actuator::to_servo_pulse_us(cfg, -90.0f * PI / 180.0f), 950,
		"-90 deg clamps to min");

	// Non-finite input -> centre, not a crash.
	expect_pulse(actuator::to_servo_pulse_us(cfg, std::nan("")), 1475,
		"NaN -> centre");
	expect_pulse(
		actuator::to_servo_pulse_us(cfg, std::numeric_limits<float>::infinity()),
		1475, "inf -> centre");

	// Half command on each side. The span is asymmetric about centre
	// (1475->2000 is 525 us, 1475->950 is 525 us here), so half-command is
	// centre +/- 262.5 -> rounds to 1738 / 1213.
	expect_pulse(actuator::to_servo_pulse_us(cfg, 22.5f * PI / 180.0f), 1738,
		"+22.5 deg -> 1738");
	expect_pulse(actuator::to_servo_pulse_us(cfg, -22.5f * PI / 180.0f), 1213,
		"-22.5 deg -> 1213");

	// Sign inversion swaps left/right.
	actuator::ServoPulseConfig inverted = cfg;
	inverted.steering_to_servo_sign = -1.0f;
	expect_pulse(actuator::to_servo_pulse_us(inverted, 45.0f * PI / 180.0f), 950,
		"inverted +45 deg -> min");

	// --- limit_servo_pulse_step ---------------------------------------------
	// Within the step budget: passes through.
	expect_pulse(actuator::limit_servo_pulse_step(cfg, 1600, 1475), 1600,
		"125 us step passes");
	// Beyond the budget: clamps to +/- maximum_servo_step_us (500).
	expect_pulse(actuator::limit_servo_pulse_step(cfg, 2000, 1475), 1975,
		"525 us up -> +500");
	expect_pulse(actuator::limit_servo_pulse_step(cfg, 950, 1475), 975,
		"525 us down -> -500");
	// No movement.
	expect_pulse(actuator::limit_servo_pulse_step(cfg, 1475, 1475), 1475,
		"no step");

	if (failures == 0) {
		std::cout << "PASS: servo pulse golden values\n";
		return 0;
	}
	std::cerr << failures << " servo-pulse assertion(s) failed\n";
	return 1;
}
