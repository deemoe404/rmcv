//
// Created by Sam Cheung on 7/26/24.
//

#include "rmcv.h"
#include "rmcv_hardware.h"

struct serial_package
{
    int camp;
    float pitch, yaw, roll;
};

struct frame_package
{
    int64 timestamp;
    serial_package package;
    cv::Mat image;
};

int main()
{
    rm::parallel_queue<serial_package> serial_queue;
    std::thread serial_thread([&serial_queue]()
    {
        rm::serial_port serial;
        bool status = serial.initialize("/dev/ttyUSB0", B230400);

        int error_counter = 0;
        while (status)
        {
            if (error_counter > 10)
            {
                serial.destroyed();
                status = serial.initialize("/dev/ttyUSB0", B230400);
            }

            unsigned char buffer[256];
            serial.receive(buffer, 20);

            if (buffer[0] != 0x38 ||
                buffer[19] != rm::lookup_CRC(buffer, 19))
            {
                error_counter++;
                continue;
            }

            const int camp = buffer[1] & 0x01;

            float pitch, yaw, roll;
            std::memcpy(&yaw, buffer + 3, sizeof(float));
            std::memcpy(&pitch, buffer + 11, sizeof(float));
            std::memcpy(&roll, buffer + 15, sizeof(float));

            if (!serial_queue.empty()) serial_queue.tryPop();
            serial_queue.push({camp, pitch, yaw, roll});
        }
    });

    rm::parallel_queue<frame_package> frame_queue;
    std::thread frame_thread([&frame_queue, &serial_queue]()
    {
        rm::hardware::daheng camera;
        const bool status = camera.initialize("KE0210010004", false, 4000, 1);

        while (status)
        {
            cv::Mat image = camera.capture(true, true);
            if (image.empty()) break;

            auto package = serial_queue.pop();
            frame_queue.push({cv::getTickCount(), *package, image});
        }
    });

    // process frame thread


    // write serial message thread
}
