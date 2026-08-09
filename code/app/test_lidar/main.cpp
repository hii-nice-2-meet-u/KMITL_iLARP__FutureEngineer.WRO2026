#include <chrono>
#include <iostream>
#include <thread>

#include "lidar_module.hpp"

int main()
{
    lidar::LidarModule lidar;

    if (!lidar.initialize()) {
        std::cerr << "Initialize failed\n";
        return -1;
    }

    if (!lidar.start()) {
        std::cerr << "Start failed\n";
        return -1;
    }

    std::cout << "Scanning for 10 seconds...\n";

    std::this_thread::sleep_for(std::chrono::seconds(10));

    lidar.stop();
    lidar.shutdown();

    std::cout << "Finished\n";

    return 0;
}