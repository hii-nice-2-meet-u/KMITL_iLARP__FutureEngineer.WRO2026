// test_lidar.cpp
// Full test program for Slamtec RPLiDAR SDK v2.1.0
// Compile: g++ -std=c++11 test_lidar.cpp -I/path/to/sdk/include -L/path/to/sdk/lib -lrplidar_sdk -pthread -o test_lidar
//
// Or compile with SDK source:
// g++ -std=c++11 test_lidar.cpp /path/to/sdk/src/*.cpp /path/to/sdk/src/arch/linux/*.cpp /path/to/sdk/src/hal/*.cpp -I/path/to/sdk/include -I/path/to/sdk/src -pthread -o test_lidar

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;

static volatile bool ctrl_c_pressed = false;

void on_ctrl_c(int)
{
    ctrl_c_pressed = true;
}

void print_usage(const char* prog)
{
    printf("Usage: %s [serial_port] [baudrate]\n", prog);
    printf("  Default: /dev/ttyUSB0 @ 115200 baud\n");
    printf("  Example: %s /dev/ttyUSB0 115200\n", prog);
    printf("  Example: %s /dev/ttyACM0 256000   (for A2/A3 models)\n", prog);
}

template<typename T>
static T* unwrap_ptr(T* ptr) { return ptr; }


int main(int argc, const char * argv[])
{
    // --- Parse arguments ---
    const char * serial_port = "/dev/ttyUSB0";
    sl_u32 baudrate = 1000000;

    if (argc > 1) {
        if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        serial_port = argv[1];
    }
    if (argc > 2) baudrate = strtoul(argv[2], NULL, 10);

    printf("=================================================\n");
    printf("  RPLiDAR Test Tool\n");
    printf("  SDK Version: %s\n", SL_LIDAR_SDK_VERSION);
    printf("=================================================\n");
    printf("Port     : %s\n", serial_port);
    printf("Baudrate : %u\n", baudrate);
    printf("-------------------------------------------------\n\n");

    auto channel_raw= createSerialPortChannel(serial_port, baudrate);
    IChannel* channel = unwrap_ptr(*channel_raw);

    if (!channel) {
        fprintf(stderr, "[ERROR] Failed to create serial channel for %s\n", serial_port);
        fprintf(stderr, "        Check port permissions (sudo chmod 666 %s)\n", serial_port);
        return -1;
    }

    // --- Create LiDAR driver ---
    auto lidar_raw = createLidarDriver();
    ILidarDriver* lidar = unwrap_ptr(*lidar_raw);

    if (!lidar) {
        fprintf(stderr, "[ERROR] Failed to create lidar driver\n");
        delete channel;
        return -1;
    }

    // --- Connect ---
    sl_result res = lidar->connect(channel);
    if (!SL_IS_OK(res)) {
        fprintf(stderr, "[ERROR] Failed to connect to LIDAR: 0x%08x\n", res);
        fprintf(stderr, "        - Check USB cable and power\n");
        fprintf(stderr, "        - Verify baudrate matches your model\n");
        fprintf(stderr, "        - Check dmesg | grep tty\n");
        delete lidar;
        delete channel;
        return -1;
    }
    printf("[OK] Connected to LIDAR successfully.\n\n");

    // --- Get Device Info ---
    sl_lidar_response_device_info_t devInfo;
    res = lidar->getDeviceInfo(devInfo);
    if (SL_IS_OK(res)) {
        printf("--- Device Information ---\n");
        printf("  Model         : %d", devInfo.model);
        switch (devInfo.model) {
            case 0: printf(" (A1)"); break;
            case 1: printf(" (A2)"); break;
            case 2: printf(" (A3)"); break;
            case 3: printf(" (S1)"); break;
            case 6: printf(" (C1)"); break;
            default: break;
        }
        printf("\n");
        printf("  Firmware Ver  : %d.%d\n", 
               devInfo.firmware_version >> 8, 
               devInfo.firmware_version & 0xFF);
        printf("  Hardware Ver  : %d\n", devInfo.hardware_version);
        printf("  Serial Number : ");
        for (int i = 0; i < 16; ++i) printf("%02X", devInfo.serialnum[i]);
        printf("\n\n");
    } else {
        fprintf(stderr, "[WARN] Failed to get device info: 0x%08x\n\n", res);
    }

    // --- Get Health Status ---
    sl_lidar_response_device_health_t health;
    res = lidar->getHealth(health);
    if (SL_IS_OK(res)) {
        printf("--- Health Status ---\n");
        printf("  Status     : ");
        switch (health.status) {
            case SL_LIDAR_STATUS_OK:       printf("OK (0)"); break;
            case SL_LIDAR_STATUS_WARNING:  printf("WARNING (1)"); break;
            case SL_LIDAR_STATUS_ERROR:    printf("ERROR (2)"); break;
            default:                       printf("UNKNOWN (%d)", health.status); break;
        }
        printf("\n");
        printf("  Error Code : 0x%04x\n\n", health.error_code);

        if (health.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "[WARN] LIDAR reports a hardware error. Continuing anyway...\n\n");
        }
    } else {
        fprintf(stderr, "[WARN] Failed to get health status: 0x%08x\n\n", res);
    }

    // --- Optional: Set motor speed (uncomment if your model needs it) ---
    // lidar->setMotorSpeed(DEFAULT_MOTOR_PWM);

    // --- Start Scan ---
    printf("--- Starting Scan ---\n");
    res = lidar->startScan(false, true);  // force=false, useTypicalScan=true
    if (!SL_IS_OK(res)) {
        fprintf(stderr, "[ERROR] Failed to start scan: 0x%08x\n", res);
        lidar->disconnect();
        delete lidar;
        delete channel;
        return -1;
    }

    signal(SIGINT, on_ctrl_c);
    printf("[OK] Scan started. Press Ctrl+C to stop.\n");
    printf("=================================================\n");
    printf("  Scan #  |  Points  |  Min Dist  |  Max Dist   \n");
    printf("=================================================\n");

    // --- Scan Loop ---
    int scan_count = 0;
    while (!ctrl_c_pressed) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        res = lidar->grabScanDataHq(nodes, count);
        if (SL_IS_OK(res)) {
            // Sort data by angle (ascending)
            lidar->ascendScanData(nodes, count);

            ++scan_count;

            // Calculate statistics
            float min_dist = 999999.0f;
            float max_dist = 0.0f;
            int valid_points = 0;

            for (size_t i = 0; i < count; ++i) {
                float distance = nodes[i].dist_mm_q2 / 4.0f;
                if (distance > 0.0f) {
                    if (distance < min_dist) min_dist = distance;
                    if (distance > max_dist) max_dist = distance;
                    ++valid_points;
                }
            }

            // Print summary every scan
            printf("  %5d   |  %4zu   |  %7.1f   |  %7.1f mm\n",
                   scan_count, count, min_dist, max_dist);

            // --- DEBUG: Print first 3 points in detail ---
            // Uncomment below to see individual point data
            
            for (size_t i = 0; i < (count < 3 ? count : 3); ++i) {
                float angle = nodes[i].angle_z_q14 * 90.f / 16384.f;
                float distance = nodes[i].dist_mm_q2 / 4.0f;
                printf("    [%3zu] Angle: %6.2f deg | Dist: %8.2f mm | Quality: %u\n",
                       i, angle, distance, (unsigned)nodes[i].quality);
            }
            

            fflush(stdout);
        } else {
            // Non-fatal errors: just skip this frame
            if (res != SL_RESULT_OPERATION_TIMEOUT && res != SL_RESULT_OPERATION_FAIL) {
                fprintf(stderr, "\n[WARN] grabScanDataHq returned: 0x%08x\n", res);
            }
        }
    }


    lidar->stop();
    lidar->disconnect();

    delete lidar;
    delete channel;

    printf("Cleanup complete.");
    return 0;
}