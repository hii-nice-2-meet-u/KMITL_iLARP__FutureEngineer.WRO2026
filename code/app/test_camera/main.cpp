#include <iostream>
#include <opencv2/opencv.hpp>
#include "lccv.hpp"
#include "libcamera/controls.h"

int main() {
    lccv::PiCamera cam;
    cam.options->video_width = 640;
    cam.options->video_height = 480;
    cam.options->framerate = 30;
    cam.options->verbose = true;

    libcamera::ControlList &controls = cam.getControlList();
    controls.set(libcamera::controls::AfMode, libcamera::controls::AfModeContinuous);

    if (!cam.startVideo()) {
        std::cerr << "Failed to start video!" << std::endl;
        return -1;
    }

    cv::Mat frame;
    cv::namedWindow("LCCV Stream", cv::WINDOW_NORMAL);

    while (true) {
        if (cam.getVideoFrame(frame, 1000)) {
            cv::imshow("LCCV Stream", frame);
        }

        if (cv::waitKey(1) == 'q') break;
    }

    cam.stopVideo();
    cv::destroyAllWindows();
    return 0;
}