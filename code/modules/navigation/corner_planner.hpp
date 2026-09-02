#pragma once

#include <algorithm>
#include <cmath>

#include "kinematics.hpp"

namespace navigation {

// Re-planned corner controller (corner-strategy redesign, phase C-2).
//
// The legacy TURNING mode follows a clothoid-arc reference whose radius is the
// fixed config `corner_radius_m` and whose shape is latched at the trigger tick;
// the measured corner point is discarded the moment the turn begins. This
// planner instead steers, EVERY tick, along the arc that passes the *measured*
// corner point at a fixed path offset. A late or early corner estimate therefore
// self-corrects continuously instead of being frozen into a blind arc.
//
// The curvature law is pure pursuit to an apex point: the corner point pulled
// toward the track interior by `path_offset_m`. Pure pursuit to a point is exact
// and unambiguous; only the apex *placement* (the offset sign/magnitude) encodes
// the racing line and must be validated in the sim / on track before the planner
// is enabled. That is why the caller keeps it behind a default-off flag until
// M-4 (curvature_gain) and a C-0 baseline exist.
//
// Frame convention matches the controller everywhere: robot +X = right,
// +Y = forward; curvature > 0 curves right, < 0 curves left.
struct CornerPlan {
	float curvature_1pm{0.0f};
	float apex_forward_m{0.0f}; // apex in the robot frame, for telemetry
	float apex_lateral_m{0.0f};
	bool valid{false};
};

class CornerPlanner {
  public:
	// corner_world_{x,y} : confirmed corner point in the OTOS world frame.
	// pose_*             : current OTOS pose (world position + heading).
	// path_offset_m      : distance from the corner point to the path apex.
	// interior_sign      : +1 for a clockwise corner (inner wall on the right,
	//                      path passes to its left), -1 for counter-clockwise.
	//                      Derived from DrivingDirection by the caller.
	static CornerPlan plan(float corner_world_x, float corner_world_y,
		float pose_x, float pose_y, float pose_heading, float path_offset_m,
		float interior_sign, const kinematics::BicycleModel &model,
		float max_steering_rad) {
		CornerPlan out;

		// World -> robot frame. This is the inverse of the forward transform the
		// controller uses to place the landmark, so apex_* read back in the same
		// units as wall_corner_forward_m / wall_corner_lateral_m.
		const float dx = corner_world_x - pose_x;
		const float dy = corner_world_y - pose_y;
		const float cosine = std::cos(pose_heading);
		const float sine = std::sin(pose_heading);
		const float corner_forward_m = -dx * sine + dy * cosine;
		const float corner_lateral_m = dx * cosine + dy * sine;

		// Apex = corner point shifted toward the track interior by the offset.
		const float apex_forward_m = corner_forward_m;
		const float apex_lateral_m =
			corner_lateral_m - interior_sign * std::max(0.0f, path_offset_m);
		out.apex_forward_m = apex_forward_m;
		out.apex_lateral_m = apex_lateral_m;

		// The apex must be ahead of the vehicle for a pursuit solution to exist.
		const float squared_distance =
			apex_forward_m * apex_forward_m + apex_lateral_m * apex_lateral_m;
		if (apex_forward_m <= 0.02f || squared_distance < 1e-6f) {
			return out; // invalid: caller falls back to the legacy reference
		}

		// Pure-pursuit curvature of the circle through the origin, tangent to the
		// current heading (+Y), passing through the apex.
		float curvature_1pm = 2.0f * apex_lateral_m / squared_distance;
		const float max_curvature = model.max_curvature(max_steering_rad);
		curvature_1pm = std::clamp(curvature_1pm, -max_curvature, max_curvature);

		out.curvature_1pm = curvature_1pm;
		out.valid = true;
		return out;
	}
};

} // namespace navigation
