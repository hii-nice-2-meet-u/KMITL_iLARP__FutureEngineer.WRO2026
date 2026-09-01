#include <atomic>
#include <cmath>
#include <csignal>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "camera_module.hpp"
#include "camera_processor.hpp"
#include "lidar_module.hpp"
#include "lidar_processor.hpp"
#include "obstacle_controller.hpp"
#include "open_challenge_common.hpp"
#include "perception.hpp"

namespace {
constexpr int SIZE = 640;
constexpr float PPM = 185.0f;
constexpr float RAD_TO_DEG = 57.2957795f;
std::atomic_bool stop_requested{false};
void stop(int) { stop_requested.store(true); }
cv::Point pixel(float right, float forward) {
	return {SIZE / 2 + static_cast<int>(std::lround(right * PPM)),
		SIZE - 30 - static_cast<int>(std::lround(forward * PPM))};
}
cv::Point raw_pixel(const LidarPoint &p) {
	const float a = p.angle_deg / RAD_TO_DEG;
	return pixel(-p.distance_m * std::sin(a), -p.distance_m * std::cos(a));
}
cv::Scalar color(camera::Color c) {
	return c == camera::Color::Red ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
}
void line(cv::Mat &v, const lidar::LineSegment &s, cv::Scalar c, int width) {
	cv::line(v, pixel(s.start.x, s.start.y), pixel(s.end.x, s.end.y), c, width, cv::LINE_AA);
}
void arrow(cv::Mat &v, const obstacle_challenge::ObstacleStatus &s, bool camera) {
	if (!s.active || !s.pass_side) return;
	const bool right = *s.pass_side == navigation::PassSide::RIGHT;
	const std::string text = right ? "AVOID RIGHT" : "AVOID LEFT";
	const cv::Point from = camera ? cv::Point(v.cols / 2, v.rows - 45) : pixel(0, 0);
	const cv::Point to = camera
		? cv::Point(v.cols / 2 + (right ? 150 : -150), v.rows - 150)
		: pixel(s.target_right_m, std::max(0.30f, s.forward_m));
	cv::arrowedLine(v, from, to, cv::Scalar(0, 255, 255), 7, cv::LINE_AA, 0, 0.24);
	cv::putText(v, text, {15, 48}, cv::FONT_HERSHEY_SIMPLEX, 0.9,
		cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
}
void draw_lidar(cv::Mat &v, const TimedLidarData &scan,
	const lidar::ProcessedLidarData &data, const perception::PerceptionData &fused,
	const obstacle_challenge::ObstacleStatus &status) {
	v.setTo(cv::Scalar(12, 12, 12));
	for (const auto &p : scan.points) {
		if (p.quality < 10 || p.distance_m <= 0.015f) continue;
		const auto q = raw_pixel(p);
		if (q.x >= 0 && q.x < v.cols && q.y >= 0 && q.y < v.rows)
			cv::circle(v, q, 1, cv::Scalar(85, 85, 85), -1);
	}
	for (const auto &s : data.line_segments) line(v, s, cv::Scalar(110, 110, 110), 1);
	if (data.walls.left) line(v, *data.walls.left, cv::Scalar(255, 100, 0), 3);
	if (data.walls.right) line(v, *data.walls.right, cv::Scalar(255, 100, 0), 3);
	if (data.walls.front) line(v, *data.walls.front, cv::Scalar(0, 255, 255), 3);
	for (const auto &o : fused.obstacles) {
		const auto c = o.camera_color ? color(*o.camera_color) : cv::Scalar(255, 255, 0);
		cv::circle(v, pixel(o.robot_position.right_m, o.robot_position.forward_m),
			o.frame_confirmed ? 11 : 7, c, 2, cv::LINE_AA);
	}
	arrow(v, status, false);
	cv::circle(v, pixel(0, 0), 7, cv::Scalar(255, 255, 255), -1);
}
void draw_camera(cv::Mat &v, const camera::ProcessedCameraData &data,
	const perception::PerceptionData &fused,
	const obstacle_challenge::ObstacleStatus &status) {
	for (std::size_t i = 0; i < data.objects.size(); ++i) {
		const auto &o = data.objects[i];
		bool confirmed = false;
		for (const auto &f : fused.obstacles)
			confirmed |= f.camera_object_index && *f.camera_object_index == i && f.frame_confirmed;
		const auto c = confirmed ? color(o.color) : cv::Scalar(0, 165, 255);
		cv::rectangle(v, o.bounding_box, c, confirmed ? 3 : 2, cv::LINE_AA);
		cv::putText(v, o.color == camera::Color::Red ? "RED" : "GREEN",
			{o.bounding_box.x, std::max(20, o.bounding_box.y - 7)},
			cv::FONT_HERSHEY_SIMPLEX, 0.6, c, 2, cv::LINE_AA);
	}
	arrow(v, status, true);
}
}

int main() {
	std::signal(SIGINT, stop); std::signal(SIGTERM, stop);
	perception::PerceptionConfig pc;
	pc.lidar_mount = {0.0f, 0.081875f, 0.0f}; pc.camera_mount = {0, 0, 0};
	obstacle_challenge::ObstacleConfig oc;
	camera::CameraModule camera(640, 640, 90.0f, 1.8f, 2.8f);
	camera::CameraProcessor camera_processor;
	lidar::LidarModule lidar("/dev/ttyAMA0", 1'000'000);
	lidar::LidarProcessor lidar_processor;
	perception::Perception fusion(pc);
	otos::OTOS otos;
	obstacle_challenge::ObstacleController controller(oc);
	if (!camera.start() || !lidar.initialize() || !lidar.start() || !otos.initialize(1)) {
		std::cerr << "Monitor initialization failed\n"; camera.stop(); lidar.stop(); return 1;
	}
	otos.setLinearUnit(kSfeOtosLinearUnitMeters);
	otos.setAngularUnit(kSfeOtosAngularUnitRadians);
	if (!open_challenge::calibrate_otos(otos)) { camera.stop(); lidar.stop(); return 1; }
	std::cout << "Obstacle monitor: NO SPI; Q/ESC stops\n";
	cv::Mat lidar_view(SIZE, SIZE, CV_8UC3);
	while (!stop_requested.load()) {
		TimedLidarData scan;
		if (!lidar.wait_for_data(scan) || scan.points.empty()) break;
		sfe_otos_pose2d_t p{}, velocity{}, acceleration{};
		if (otos.getPosVelAcc(p, velocity, acceleration) != ksfTkErrOk) break;
		const navigation::MapPose pose{p.x, p.y, p.h};
		const auto lidar_data = open_challenge::process_scan(lidar_processor, scan, 0.0f);
		TimedFrameData frame; camera::ProcessedCameraData camera_data; cv::Mat camera_view;
		if (camera.get_latest(frame) && !frame.frame.empty()) {
			camera_data = camera_processor.process(frame); camera_view = frame.frame.clone();
		}
		const auto fused = fusion.process(lidar_data, camera_data, pose);
		navigation::NavigationCommand command; command.target_speed_mps = oc.avoidance_speed_mps;
		const auto status = controller.apply(fused, pose, navigation::NavigationMode::NORMAL, {}, false, command);
		draw_lidar(lidar_view, scan, lidar_data, fused, status);
		cv::imshow("Obstacle Monitor - LiDAR", lidar_view);
		if (!camera_view.empty()) { draw_camera(camera_view, camera_data, fused, status); cv::imshow("Obstacle Monitor - Camera", camera_view); }
		std::cout << "[MONITOR] fused=" << fused.obstacles.size()
			<< " active=" << (status.active ? "YES" : "NO")
			<< " pass=" << (status.pass_side ? obstacle_challenge::side_name(*status.pass_side) : "-")
			<< " distance=" << status.forward_m << "m steering="
			<< status.steering_rad * RAD_TO_DEG << "deg\r" << std::flush;
		const int key = cv::waitKey(1) & 0xff;
		if (key == 'q' || key == 'Q' || key == 27) break;
	}
	std::cout << '\n'; cv::destroyAllWindows(); lidar.stop(); camera.stop(); return 0;
}
