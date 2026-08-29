#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "camera_processor.hpp"
#include "lidar_processor.hpp"
#include "track_map.hpp"

namespace perception {

// Sensor mounting values use the robot frame used by the LiDAR dashboard:
// +right_m points to the robot's right and +forward_m points forward.
// Bearing is positive to the right, matching CameraObject::bearing_rad and
// ObstacleObject::bearing_rad(). Mount yaw is also positive to the right.
struct SensorMount {
	float right_m{0.0f};
	float forward_m{0.0f};
	float yaw_rad{0.0f};
};

struct PerceptionConfig {
	// A camera observation is never fused with a LiDAR observation when their
	// capture times differ by more than this amount.
	std::uint64_t max_sensor_time_difference_us{100'000};

	// Maximum angular separation allowed for a camera/LiDAR association.
	float max_bearing_difference_rad{8.0f * 3.14159265358979323846f / 180.0f};

	// Final fusion confidence required for a camera/LiDAR pair to be accepted
	// in the current frame. This is not temporal tracking: a caller should
	// still require repeated frame-confirmed observations before committing a
	// map landmark. LiDAR-only objects remain in the output but are never
	// confirmed.
	float minimum_confirmed_confidence{0.55f};

	// Sanity gates applied in addition to LidarProcessor's own clustering
	// filters. They reject NaN/Inf and physically implausible input before
	// matching.
	float minimum_lidar_distance_m{0.05f};
	float maximum_lidar_distance_m{3.00f};

	SensorMount lidar_mount;
	SensorMount camera_mount;
};

struct RobotPoint {
	float right_m{0.0f};
	float forward_m{0.0f};
};

struct WorldPoint {
	float x_m{0.0f};
	float y_m{0.0f};
};

enum class ObservationSource {
	LIDAR_ONLY,
	LIDAR_CAMERA_FUSED,
};

struct FusedObstacle {
	ObservationSource source{ObservationSource::LIDAR_ONLY};
	std::size_t lidar_object_index{0};
	std::optional<std::size_t> camera_object_index;

	RobotPoint robot_position;
	std::optional<WorldPoint> world_position;
	float distance_m{0.0f};
	float width_m{0.0f};
	float lidar_bearing_rad{0.0f};

	std::optional<camera::Color> camera_color;
	std::optional<navigation::TrafficColor> traffic_color;
	std::optional<navigation::PassSide> required_pass_side;
	float camera_bearing_rad{0.0f};
	float predicted_camera_bearing_rad{0.0f};
	float bearing_error_rad{0.0f};
	float fusion_confidence{0.0f};
	// True only for this sensor pair in this frame. It does not mean the same
	// physical object has been observed over multiple frames.
	bool frame_confirmed{false};
};

struct PerceptionDiagnostics {
	std::uint64_t lidar_timestamp_us{0};
	std::uint64_t camera_timestamp_us{0};
	std::uint64_t sensor_time_difference_us{0};
	bool pose_valid{false};
	bool camera_time_synchronized{false};

	std::size_t lidar_input_count{0};
	std::size_t valid_lidar_count{0};
	std::size_t rejected_lidar_count{0};
	std::size_t camera_input_count{0};
	std::size_t valid_camera_count{0};
	std::size_t rejected_camera_count{0};
	std::size_t matched_count{0};
	std::size_t frame_confirmed_count{0};
	std::size_t unmatched_lidar_count{0};
	std::size_t unmatched_camera_count{0};
};

struct PerceptionData {
	std::uint64_t timestamp_us{0};
	lidar::ResolvedWalls track_walls;
	std::optional<lidar::LineSegment> parking_wall;
	std::vector<FusedObstacle> obstacles;

	// Indexes refer to ProcessedCameraData::objects. Keeping unmatched camera
	// detections visible makes test_combined able to explain why an object was
	// not fused instead of silently discarding it.
	std::vector<std::size_t> unmatched_camera_object_indices;
	PerceptionDiagnostics diagnostics;
};

class Perception {
  public:
	explicit Perception(PerceptionConfig config = {});

	PerceptionData process(const lidar::ProcessedLidarData &lidar_data,
		const camera::ProcessedCameraData &camera_data,
		const navigation::MapPose &vehicle_pose) const;

	const PerceptionConfig &config() const { return config_; }

  private:
	static float normalize_angle(float angle_rad);
	static std::uint64_t timestamp_difference(
		std::uint64_t a_us, std::uint64_t b_us);
	static bool valid_pose(const navigation::MapPose &pose);
	static navigation::TrafficColor to_traffic_color(camera::Color color);
	static navigation::PassSide pass_side_for(camera::Color color);

	bool valid_lidar_object(const lidar::ObstacleObject &object) const;
	static bool valid_camera_object(const camera::CameraObject &object);
	RobotPoint lidar_to_robot(const cv::Point2f &point) const;
	std::optional<WorldPoint> robot_to_world(
		const RobotPoint &point, const navigation::MapPose &pose) const;
	float predicted_camera_bearing(const RobotPoint &point) const;

	PerceptionConfig config_;
};

} // namespace perception
