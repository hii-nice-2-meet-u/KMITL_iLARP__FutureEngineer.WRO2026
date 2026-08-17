#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "camera_module.hpp"
#include "camera_processor.hpp"

int main() {
	const std::string folder_path = "/home/hii/captured_wro/test_camera/";

	const std::string mkdir_cmd = "mkdir -p " + folder_path;

	system(mkdir_cmd.c_str());

	// -------------------------------------------------------------------------
	// Camera
	// -------------------------------------------------------------------------

	camera::CameraModule camera_module(640, 640, 90, 1.8, 2.8);

	if (!camera_module.start()) {
		std::cerr << "Failed to start camera!\n";
		return 1;
	}

	camera::CameraProcessor camera_processor;

	int image_count = 0;

	while (true) {

		TimedFrameData frame_data;

		if (!camera_module.wait_for_frame(frame_data)) {
			std::cerr << "Failed to get camera frame.\n";
			break;
		}

		if (frame_data.frame.empty()) {
			continue;
		}

		// Process camera frame
		const camera::ProcessedCameraData processed =
			camera_processor.process(frame_data);

		// Clone because this image is only for debug drawing.
		cv::Mat display_frame = frame_data.frame.clone();

		for (const auto &object : processed.objects) {

			cv::Scalar draw_color;

			std::string color_name;

			switch (object.color) {

			case camera::Color::Red:
				draw_color = cv::Scalar(0, 0, 255);
				color_name = "RED";
				break;

			case camera::Color::Green:
				draw_color = cv::Scalar(0, 255, 0);
				color_name = "GREEN";
				break;
			}

			cv::rectangle(display_frame, object.bounding_box, draw_color, 2);

			cv::circle(display_frame, object.bottom_center, 5, draw_color, -1);

			const std::string text1 =
				color_name + " bearing: " + std::to_string(object.bearing_deg);
			const std::string text2 = color_name +
				" width: " + std::to_string(object.bounding_box.width) +
				", height: " + std::to_string(object.bounding_box.height);

			cv::putText(display_frame, text1,
				cv::Point(object.bounding_box.x,
					std::max(20, object.bounding_box.y - 10)),
				cv::FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2);
			cv::putText(display_frame, text2,
				cv::Point(object.bounding_box.x - 5,
					std::max(20, object.bounding_box.y - 30)),
				cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(153, 153, 0), 2);

			std::cout << "Color: " << color_name
					  << " | x: " << object.bounding_box.x
					  << " | y: " << object.bounding_box.y
					  << " | w: " << object.bounding_box.width
					  << " | h: " << object.bounding_box.height
					  << " | bearing: " << object.bearing_deg << '\n';
		}

		std::cout << "Timestamp: " << processed.timestamp_us
				  << " | Objects: " << processed.objects.size() << '\n';

		cv::imshow("Camera Processor Test", display_frame);

		const char key = static_cast<char>(cv::waitKey(1));

		if (key == 's') {

			const std::string file_path =
				folder_path + "img_" + std::to_string(image_count) + ".jpg";

			if (cv::imwrite(file_path, frame_data.frame)) {

				std::cout << "Saved: " << file_path << '\n';

			} else {

				std::cerr << "Error saving image to " << file_path << '\n';
			}

			++image_count;
		}

		if (key == 'q') {
			camera_module.stop();
			break;
		}
	}

	camera_module.stop();

	cv::destroyAllWindows();

	return 0;
}