#pragma once

#include <cstddef>
#include <optional>

#include "direction.hpp"

namespace navigation {

enum class NavigationMode { SEARCH_DIRECTION, NORMAL, TURNING, FINISHED };

struct NavigationCommand {
	// Normalized command
	// speed:    0 -> stop, 100 -> max forward
	// steering: 1000.0 -> left, 2100.0 -> right
	float speed{0.0f};
	float steering_rad{0.0f};
};

struct NavigationDebug {
	// Wall following
	float outer_distance_m{0.0f};
	float distance_error_m{0.0f};

	float wall_angle_rad{0.0f};
	float angle_error_rad{0.0f};

	// Turning
	float heading_error_rad{0.0f};

	bool outer_wall_valid{false};
	bool front_wall_valid{false};
};

struct NavigationState {
	NavigationMode mode{NavigationMode::SEARCH_DIRECTION};

	std::optional<DrivingDirection> direction;

	// Heading target of current straight / turn
	float target_heading_rad{0.0f};

	// Turn state
	bool turn_armed{true};
	int heading_confirm_frames{0};

	// Track progress
	int turn_count{0};
	int lap{0};
	std::size_t corner_index{0};
};

struct NavigationResult {
	NavigationCommand command;
	NavigationDebug debug;
};

} // namespace navigation