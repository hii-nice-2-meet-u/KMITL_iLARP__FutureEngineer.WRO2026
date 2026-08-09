#include <iostream>
#include <sstream>
#include <iomanip>
#include <random>
#include <chrono>

#include <opencv2/opencv.hpp>
#include "lccv.hpp"
#include "libcamera/controls.h"
#include "opencv2/imgcodecs.hpp"

std::string generateHexID() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> dis(0, 0xFFFFFFFF);
    
    std::stringstream ss;
    ss << std::hex << std::setw(8) << std::setfill('0') << dis(gen);
    return ss.str();
}

int main() {
    //
    std::string folder_path = "/home/hii/captured_wro/test_bb/";

    std::string mkdir_cmd = "mkdir -p " + folder_path;
    system(mkdir_cmd.c_str());

    //init cam
    lccv::PiCamera cam;
    cam.options->video_width = 640;
    cam.options->video_height = 640;
    cam.options->framerate = 100;
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
            cv::flip(frame, frame, 0); 
            cv::imshow("LCCV Stream", frame);
        }
        if (cv::waitKey(1) == 's') {
            std::string hex_id = generateHexID();
            std::string file_path = folder_path + "img_" + hex_id + ".jpg";

            if (cv::imwrite(file_path, frame)) {
                std::cout << "Saved: " << file_path << std::endl;
            } else {
                std::cerr << "Error saving image to " << file_path << std::endl;
            }
        }
        

        if (cv::waitKey(1) == 'q') break;
    }

    cam.stopVideo();
    cv::destroyAllWindows();
    return 0;
}