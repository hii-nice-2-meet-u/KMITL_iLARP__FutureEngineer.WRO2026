#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "camera_module.hpp"
#include "camera_processor.hpp"
#include "lidar_module.hpp"
#include "lidar_processor.hpp"
#include "perception.hpp"

namespace {

constexpr int VIEW_SIZE = 640;
constexpr float PIXELS_PER_METER = 185.0f;
constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;

std::atomic_bool stop_requested{false};

void request_stop(int) { stop_requested.store(true); }

cv::Point robot_to_pixel(float right_m, float forward_m) {
	return {VIEW_SIZE / 2 +
			static_cast<int>(std::lround(right_m * PIXELS_PER_METER)),
		VIEW_SIZE - 30 -
			static_cast<int>(std::lround(forward_m * PIXELS_PER_METER))};
}

cv::Point raw_lidar_to_pixel(const LidarPoint &point) {
	const float rad = point.angle_deg * 3.14159265358979323846f / 180.0f;
	return robot_to_pixel(
		point.distance_m * -std::sin(rad), point.distance_m * -std::cos(rad));
}

std::string fixed(float value, int precision = 2) {
	std::ostringstream os;
	os << std::fixed << std::setprecision(precision) << value;
	return os.str();
}

const char *color_name(camera::Color color) {
	return color == camera::Color::Red ? "RED" : "GREEN";
}

cv::Scalar object_color(camera::Color color) {
	return color == camera::Color::Red ? cv::Scalar(0, 0, 255)
									   : cv::Scalar(0, 255, 0);
}

void draw_line_segment(cv::Mat &view, const lidar::LineSegment &segment,
	const cv::Scalar &color, int thickness) {
	cv::line(view, robot_to_pixel(segment.start.x, segment.start.y),
		robot_to_pixel(segment.end.x, segment.end.y), color, thickness,
		cv::LINE_AA);
}

void draw_lidar_view(cv::Mat &view, const TimedLidarData &scan,
	const lidar::ProcessedLidarData &processed,
	const perception::PerceptionData &fused) {
	view.setTo(cv::Scalar(12, 12, 12));
	const cv::Point origin = robot_to_pixel(0.0f, 0.0f);

	for (int meter = 1; meter <= 3; ++meter) {
		cv::circle(view, origin, static_cast<int>(meter * PIXELS_PER_METER),
			cv::Scalar(35, 35, 35), 1, cv::LINE_AA);
	}
	cv::line(view, {VIEW_SIZE / 2, 0}, {VIEW_SIZE / 2, VIEW_SIZE},
		cv::Scalar(45, 45, 45), 1);

	for (const auto &point : scan.points) {
		if (point.quality < 10 || point.distance_m <= 0.015f) {
			continue;
		}
		const cv::Point pixel = raw_lidar_to_pixel(point);
		if (pixel.x >= 0 && pixel.x < view.cols && pixel.y >= 0 &&
			pixel.y < view.rows) {
			cv::circle(view, pixel, 1, cv::Scalar(80, 80, 80), -1);
		}
	}

	for (const auto &segment : processed.line_segments) {
		draw_line_segment(view, segment, cv::Scalar(100, 100, 100), 1);
	}
	if (processed.walls.left) {
		draw_line_segment(
			view, *processed.walls.left, cv::Scalar(255, 100, 0), 3);
	}
	if (processed.walls.right) {
		draw_line_segment(
			view, *processed.walls.right, cv::Scalar(255, 100, 0), 3);
	}
	if (processed.walls.front) {
		draw_line_segment(
			view, *processed.walls.front, cv::Scalar(0, 255, 255), 3);
	}

	for (std::size_t index = 0; index < fused.obstacles.size(); ++index) {
		const auto &object = fused.obstacles[index];
		cv::Scalar color(255, 255, 0);
		if (object.camera_color) {
			color = object.frame_confirmed ? object_color(*object.camera_color)
										   : cv::Scalar(0, 165, 255);
		}
		const cv::Point center = robot_to_pixel(
			object.robot_position.right_m, object.robot_position.forward_m);
		cv::circle(view, center, object.frame_confirmed ? 10 : 7, color, 2,
			cv::LINE_AA);

		std::string label = "L" + std::to_string(object.lidar_object_index);
		if (object.camera_object_index) {
			label += "<->C" + std::to_string(*object.camera_object_index);
		}
		label += " " + fixed(object.distance_m) + "m";
		if (object.camera_color) {
			label += " " + std::string(color_name(*object.camera_color)) +
				" e=" + fixed(object.bearing_error_rad * RAD_TO_DEG, 1) +
				"deg q=" + fixed(object.fusion_confidence, 2);
		}
		cv::putText(view, label, center + cv::Point(8, -8),
			cv::FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv::LINE_AA);
	}

	cv::circle(view, origin, 7, cv::Scalar(255, 255, 255), -1);
	cv::putText(view, "LIDAR: +X RIGHT, +Y FORWARD", {10, 22},
		cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(230, 230, 230), 1,
		cv::LINE_AA);
}

void draw_camera_view(cv::Mat &view,
	const camera::ProcessedCameraData &camera_data,
	const perception::PerceptionData &fused) {
	std::vector<const perception::FusedObstacle *> matches(
		camera_data.objects.size(), nullptr);
	for (const auto &object : fused.obstacles) {
		if (object.camera_object_index &&
			*object.camera_object_index < matches.size()) {
			matches[*object.camera_object_index] = &object;
		}
	}

	for (std::size_t index = 0; index < camera_data.objects.size(); ++index) {
		const auto &camera_object = camera_data.objects[index];
		const auto *match = matches[index];
		const cv::Scalar color = match && match->frame_confirmed
			? object_color(camera_object.color)
			: cv::Scalar(0, 165, 255);
		cv::rectangle(view, camera_object.bounding_box, color,
			match && match->frame_confirmed ? 3 : 2, cv::LINE_AA);

		std::string label = "C" + std::to_string(index) + " " +
			color_name(camera_object.color) + " " +
			fixed(camera_object.bearing_rad * RAD_TO_DEG, 1) + "deg";
		if (match) {
			label += " <->L" + std::to_string(match->lidar_object_index) + " " +
				fixed(match->distance_m) +
				"m q=" + fixed(match->fusion_confidence, 2);
		} else {
			label += " UNMATCHED";
		}
		const cv::Point text_point(camera_object.bounding_box.x,
			std::max(18, camera_object.bounding_box.y - 8));
		cv::putText(view, label, text_point, cv::FONT_HERSHEY_SIMPLEX, 0.43,
			color, 1, cv::LINE_AA);
	}

	const auto &d = fused.diagnostics;
	const std::string status = "dt=" +
		fixed(static_cast<float>(d.sensor_time_difference_us) / 1000.0f, 1) +
		"ms sync=" + (d.camera_time_synchronized ? "YES" : "NO") +
		" lidar=" + std::to_string(d.valid_lidar_count) +
		" camera=" + std::to_string(d.valid_camera_count) +
		" match=" + std::to_string(d.matched_count) +
		" confirmed=" + std::to_string(d.frame_confirmed_count);
	cv::rectangle(
		view, {0, view.rows - 32, view.cols, 32}, cv::Scalar(0, 0, 0), -1);
	cv::putText(view, status, {8, view.rows - 10}, cv::FONT_HERSHEY_SIMPLEX,
		0.45,
		d.camera_time_synchronized ? cv::Scalar(0, 255, 0)
								   : cv::Scalar(0, 0, 255),
		1, cv::LINE_AA);
}

void print_diagnostics(const perception::PerceptionData &result) {
	const auto &d = result.diagnostics;
	std::cout << "[FUSION] dt=" << d.sensor_time_difference_us / 1000.0
			  << "ms sync=" << (d.camera_time_synchronized ? "YES" : "NO")
			  << " lidar=" << d.valid_lidar_count
			  << " camera=" << d.valid_camera_count
			  << " matched=" << d.matched_count
			  << " confirmed=" << d.frame_confirmed_count
			  << " unmatched(L/C)=" << d.unmatched_lidar_count << '/'
			  << d.unmatched_camera_count << '\n';
	for (const auto &object : result.obstacles) {
		std::cout << "  L" << object.lidar_object_index
				  << " distance=" << object.distance_m
				  << "m bearing=" << object.lidar_bearing_rad * RAD_TO_DEG
				  << "deg";
		if (object.camera_object_index) {
			std::cout << " <-> C" << *object.camera_object_index << ' '
					  << color_name(*object.camera_color)
					  << " error=" << object.bearing_error_rad * RAD_TO_DEG
					  << "deg confidence=" << object.fusion_confidence
					  << (object.frame_confirmed ? " CONFIRMED" : " LOW_CONF");
		} else {
			std::cout << " LIDAR_ONLY";
		}
		std::cout << '\n';
	}
}

}

int main() {
	std::signal(SIGINT, request_stop);
	std::signal(SIGTERM, request_stop);

	perception::PerceptionConfig perception_config;
	perception_config.lidar_mount = {0.0f, 0.0f, 0.0f};
	perception_config.camera_mount = {0.0f, 0.0f, 0.0f};
	perception_config.max_sensor_time_difference_us = 100'000;
	perception_config.max_bearing_difference_rad = 8.0f / RAD_TO_DEG;

	camera::CameraModule camera(640, 640, 90.0f, 1.8f, 2.8f);
	camera::CameraProcessor camera_processor;
	lidar::LidarModule lidar("/dev/ttyAMA0", 1'000'000);
	lidar::LidarProcessor lidar_processor;
	perception::Perception fusion(perception_config);

	if (!camera.start()) {
		std::cerr << "Camera initialization failed\n";
		return 1;
	}
	if (!lidar.initialize() || !lidar.start()) {
		std::cerr << "LiDAR initialization failed\n";
		camera.stop();
		return 1;
	}

	std::cout << "Real Camera + LiDAR perception test\n"
			  << "Q / ESC = quit\n"
			  << "Tune camera_mount/lidar_mount in test_perception/main.cpp\n";

	cv::Mat lidar_view(VIEW_SIZE, VIEW_SIZE, CV_8UC3);
	auto last_console_log = std::chrono::steady_clock::now();

	while (!stop_requested.load()) {
		TimedLidarData scan;
		if (!lidar.wait_for_data(scan)) {
			std::cerr << "LiDAR stream stopped\n";
			break;
		}
		if (scan.points.empty()) {
			continue;
		}

		TimedFrameData frame;
		if (!camera.get_latest(frame) || frame.frame.empty()) {
			continue;
		}

		const auto processed_lidar = lidar_processor.process(
			scan, 0.0f, 4, 0.035f, 0.12f, 5.0f, 0.04f, 0.10f);
		const auto processed_camera = camera_processor.process(frame);
		const auto result =
			fusion.process(processed_lidar, processed_camera, std::nullopt);

		cv::Mat camera_view;
		cv::resize(frame.frame, camera_view, {VIEW_SIZE, VIEW_SIZE});
		draw_camera_view(camera_view, processed_camera, result);
		draw_lidar_view(lidar_view, scan, processed_lidar, result);

		cv::Mat combined;
		cv::hconcat(camera_view, lidar_view, combined);
		cv::imshow("Real Perception Fusion: Camera | LiDAR", combined);

		const auto now = std::chrono::steady_clock::now();
		if (now - last_console_log >= std::chrono::milliseconds(500)) {
			print_diagnostics(result);
			last_console_log = now;
		}

		const int key = cv::waitKey(1);
		if (key == 'q' || key == 'Q' || key == 27) {
			break;
		}
	}

	lidar.stop();
	camera.stop();
	cv::destroyAllWindows();
	return 0;
}
