#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

int main() {
//    DahengCamera camera;
//    cv::Mat frames;
//    auto ts = (double) cv::getTickCount();
//    if (camera.dahengCameraInit((char *) "KE0210030295", 7000, 210)) {
//        while (1) {
//            ts = (double) cv::getTickCount();
//            frames = camera.getFrame();
//            cv::imshow("aaa", frames);
//
//            std::cout << 1 / ((cv::getTickCount() - ts) / cv::getTickFrequency()) << std::endl;
//            std::cout << frames.cols << std::endl;
//            std::cout << frames.rows << std::endl;
//            cv::waitKey(1);
//        }
//    }

    rm::SerialPort sp{};
    sp.Initialize();

    rm::Response response{};
    response.yaw.data = 20.0;
    response.pitch.data = 30.0;
    response.rank = 3;

    rm::Request request{};

    thread t1([&]() {
        while (true) {
            std::cout << "send status: " << sp.Send(response) << std::endl;
        }
    });

    thread t2([&]() {
        while (true) {
            if (sp.Receive(request)) {
                std::cout << "received: ";
                std::cout << "p: " << request.pitch.data << "  ";
                std::cout << "y: " << request.yaw.data << "  ";
                std::cout << "ps: " << request.pitchSpeed.data << "  ";
                std::cout << "ys: " << request.yawSpeed.data << "  ";
                std::cout << "camp: " << request.camp << "  ";
                std::cout << "mode: " << request.mode << "  ";
                std::cout << "speed: " << request.speed << std::endl;
            }
        }
    });

    t1.join();
    t2.join();

    return 0;
}
