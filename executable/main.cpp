//
// Created by Sam Cheung on 7/26/24.
//

#include "rmcv.h"
#include "rmcv_hardware.h"

struct serial_package
{
    rm::camp target;
    double pitch, yaw, roll;
};

void serial_function(rm::parallel_queue<serial_package>& serial_queue)
{
    rm::serial_port serial;
    bool status = serial.initialize("/dev/ttyUSB0", B460800);

    int error_counter = 0;
    while (status)
    {
        if (error_counter > 10)
        {
            serial.destroyed();
            status = serial.initialize("/dev/ttyUSB0", B460800);
            error_counter = 0;
        }

        unsigned char buffer[256];
        serial.receive(buffer, 24);

        if (buffer[0] != 0x38 ||
            buffer[23] != rm::lookup_CRC(buffer, 23))
        {
            error_counter++;
            continue;
        }

        const rm::camp target_camp = buffer[1] & 0x01 ? rm::camp::CAMP_RED : rm::camp::CAMP_BLUE;

        float pitch, yaw, roll;
        std::memcpy(&yaw, buffer + 3, sizeof(float));
        std::memcpy(&pitch, buffer + 11, sizeof(float));
        std::memcpy(&roll, buffer + 15, sizeof(float));

        if (!serial_queue.empty()) serial_queue.tryPop();
        serial_queue.push({
            target_camp,
            static_cast<double>(pitch) * CV_PI / 180.0f,
            static_cast<double>(yaw) * CV_PI / 180.0f,
            static_cast<double>(roll) * CV_PI / 180.0f
        });
    }
}

struct frame_package
{
    int64 timestamp;
    serial_package package;
    cv::Mat image;
};

void frame_function(rm::parallel_queue<serial_package>& serial_queue, rm::parallel_queue<frame_package>& frame_queue)
{
    rm::hardware::daheng camera;
    const bool status = camera.initialize("KE0210010004", false, 4000, 1);

    while (status)
    {
        cv::Mat image = camera.capture(true, true);
        if (image.empty()) break;

        if (!frame_queue.empty()) frame_queue.tryPop();
        auto package = serial_queue.pop();
        frame_queue.push({cv::getTickCount(), *package, image});
    }
}

void process_function(rm::parallel_queue<frame_package>& frame_queue,
                      rm::parallel_queue<std::vector<rm::armour>>& armour_queue,
                      rm::parallel_queue<cv::Mat>& debug_queue)
{
    const rm::parameters red, blue;

    while (1)
    {
        const auto frame = frame_queue.pop();
        const auto parms = frame->package.target == rm::camp::CAMP_RED ? red : blue;

        const auto contours = extract_color(frame->image, frame->package.target, 80);
        auto [positive_lightblobs, negtive_lightblobs] =
            filter_lightblobs(contours, 70, {1.5, 80}, {10, 99999},
                              frame->package.target);

        std::vector<rm::armour> armours;
        filter_armours(positive_lightblobs, armours, 12, 22, 12, frame->package.target);

        cv::Mat debug;
        frame->image.copyTo(debug);
        rm::debug::draw_lightblobs(positive_lightblobs, negtive_lightblobs, debug, -1);
        rm::debug::DrawArmours(armours, debug, -1);

        if (!debug_queue.empty()) debug_queue.tryPop();
        debug_queue.push(debug);
    }
}

int main()
{
    rm::parallel_queue<serial_package> serial_queue;
    std::thread serial_thread(serial_function, std::ref(serial_queue));

    rm::parallel_queue<frame_package> frame_queue;
    std::thread frame_thread(frame_function, std::ref(serial_queue), std::ref(frame_queue));

    rm::parallel_queue<std::vector<rm::armour>> armour_queue;
    rm::parallel_queue<cv::Mat> debug_queue;
    std::thread process_thread(process_function, std::ref(frame_queue), std::ref(armour_queue), std::ref(debug_queue));

    std::thread debug_thread([](rm::parallel_queue<cv::Mat>& debug_queue)
    {
        while (1)
        {
            const auto image = debug_queue.pop();
            cv::imshow("debug", *image);
            cv::waitKey(1);
        }
    }, std::ref(debug_queue));

    serial_thread.join();
    frame_thread.join();
    process_thread.join();
    debug_thread.join();
}
