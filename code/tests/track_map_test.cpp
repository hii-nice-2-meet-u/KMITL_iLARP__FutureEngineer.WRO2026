#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "track_map.hpp"

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

navigation::CornerObservation corner(float x_m, float y_m, float heading_rad,
	float trigger_m = 0.52f, float radius_m = 0.40f, float speed_mps = 0.65f) {
	return {{x_m, y_m, heading_rad}, trigger_m, radius_m, speed_mps};
}

} // namespace

int main() {
	navigation::TrackMap map;
	map.set_direction(DrivingDirection::CLOCKWISE);

	const navigation::CornerObservation entries[] = {
		corner(1.90f, 1.90f, 0.0f),
		corner(1.90f, 0.40f, -0.5f * PI),
		corner(0.40f, 0.40f, PI),
		corner(0.40f, 1.90f, 0.5f * PI),
	};

	const navigation::MapPose exits[] = {
		{1.90f, 1.55f, -0.5f * PI},
		{1.55f, 0.40f, PI},
		{0.40f, 0.75f, 0.5f * PI},
		{0.75f, 1.90f, 0.0f},
	};

	for (std::size_t index = 0; index < navigation::TRACK_CORNER_COUNT;
		 ++index) {
		map.record_corner_entry(index, entries[index]);
		map.record_corner_exit(index, exits[index]);

		if (index + 1 < navigation::TRACK_CORNER_COUNT) {
			expect(!map.ready_for_replay(),
				"map became ready before all four corners were learned");
		}
	}

	expect(map.ready_for_replay(),
		"map was not ready after learning four complete corners");
	expect(map.learned_corner_count() == navigation::TRACK_CORNER_COUNT,
		"learned corner count is incorrect");

	const auto far_hint = map.replay_hint({1.90f, 0.60f, 0.0f}, 0);
	expect(far_hint.has_value(), "replay hint missing for a valid map");
	expect(!far_hint->approach_recommended,
		"far replay pose requested corner approach too early");

	const auto near_hint = map.replay_hint({1.90f, 1.15f, 0.0f}, 0);
	expect(near_hint.has_value(), "near replay hint is missing");
	expect(near_hint->approach_recommended,
		"near replay pose did not request early corner approach");
	expect(std::abs(near_hint->preferred_turn_trigger_m - 0.52f) < 1e-5f,
		"replay hint did not return the learned trigger");

	// A later successful pass refines the landmark instead of replacing it.
	map.record_corner_entry(0, corner(1.94f, 1.86f, 0.02f, 0.56f));
	const auto &refined = map.corners()[0];
	expect(refined.entry_pose.x_m > 1.90f && refined.entry_pose.x_m < 1.94f,
		"corner refinement did not blend the new X position");
	expect(refined.preferred_turn_trigger_m > 0.52f &&
			refined.preferred_turn_trigger_m < 0.56f,
		"corner refinement did not blend the learned trigger");

	const std::size_t red_a = map.observe_traffic_light(1.20f, 1.55f,
		navigation::TrafficColor::RED, navigation::PassSide::RIGHT, 0.70f);
	const std::size_t red_b = map.observe_traffic_light(1.25f, 1.51f,
		navigation::TrafficColor::RED, navigation::PassSide::RIGHT, 0.80f);
	const std::size_t green = map.observe_traffic_light(0.55f, 0.90f,
		navigation::TrafficColor::GREEN, navigation::PassSide::LEFT, 0.75f);

	expect(red_a == red_b,
		"nearby observations of the same traffic light were not associated");
	expect(green != red_a, "different traffic lights were incorrectly merged");
	expect(map.traffic_landmarks().size() == 2,
		"traffic landmark count is incorrect");

	std::cout << "LEARN: 4 corner landmarks recorded\n";
	std::cout << "REPLAY: corner 0 preview at "
			  << near_hint->distance_to_entry_m << " m, learned trigger "
			  << near_hint->preferred_turn_trigger_m << " m\n";
	std::cout << "OBSTACLE MAP: 2 traffic landmarks associated\n";
	std::cout << "PASS: lap-one learning and lap-two/three replay hints\n";
	return 0;
}
