#pragma once

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include "camera_module.hpp"

namespace camera {

struct HSVRange {
	int h_min{0};
	int h_max{179};

	int s_min{0};
	int s_max{255};

	int v_min{0};
	int v_max{255};
};

class Tunner {
  public:
	Tunner(unsigned int video_width = 640, unsigned int video_height = 640,
		float framerate = 90.0f, float awb_gain_r = 1.4f,
		float awb_gain_b = 2.6f);

	~Tunner();

	Tunner(const Tunner &) = delete;
	Tunner &operator=(const Tunner &) = delete;
	void run();

	void tune_white();

	void tune_red_hsv();
	void tune_green_hsv();
	void tune_magenta_hsv();

	void stop_camera();

	[[nodiscard]] float awb_gain_r() const noexcept;
	[[nodiscard]] float awb_gain_b() const noexcept;

	[[nodiscard]] const HSVRange &red_hsv() const noexcept;
	[[nodiscard]] const HSVRange &green_hsv() const noexcept;
	[[nodiscard]] const HSVRange &magenta_hsv() const noexcept;

  private:
	bool ensure_camera_started();
	bool restart_camera();

	void tune_hsv(const std::string &tunner_window_name,
		const std::string &color_name, HSVRange &range,
		const cv::Scalar &overlay_color);

	void create_hsv_trackbars(const std::string &window_name, HSVRange &range);

	cv::Mat apply_hsv_overlay(const cv::Mat &frame, const HSVRange &range,
		const cv::Scalar &overlay_color) const;

	void draw_hsv_values(cv::Mat &frame, const HSVRange &range) const;

	void draw_awb_values(cv::Mat &frame) const;

	void show_control_panel(
		const std::string &window_name, const std::string &title) const;

	void print_hsv(const std::string &name, const HSVRange &range) const;

	void print_awb() const;

  private:
	std::unique_ptr<CameraModule> camera_module_;

	bool camera_started_{false};

	unsigned int video_width_{640};
	unsigned int video_height_{640};
	float framerate_{90.0f};

	float awb_gain_r_{1.2f};
	float awb_gain_b_{2.8f};

	// Trackbar values: real AWB gain = slider / 10.0f
	int awb_gain_r_x10_{12};
	int awb_gain_b_x10_{28};

	HSVRange red_hsv_{170, 10, 70, 255, 50, 255};

	HSVRange green_hsv_{20, 85, 180, 255, 66, 190};

	HSVRange magenta_hsv_{160, 170, 165, 255, 95, 160};

	static constexpr const char *TUNE_MASK_WINDOW = "tune_mask";
};

} // namespace camera