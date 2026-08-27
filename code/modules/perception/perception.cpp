#pragma once

struct PerceptionData {
	TrackWalls track_walls;

	std::optional<CornerEstimate> corner;
	std::optional<LineSegment> parking_wall;

	std::vector<DetectedObstacle> obstacles;

	VehicleState vehicle;
};