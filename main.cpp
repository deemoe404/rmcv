#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

int main() {

    DahengCamera camera;
    cv::Mat frames;
    auto ts = (double) cv::getTickCount();
    int key;

    if (camera.dahengCameraInit((char *) "KE0210030295", 2000, 210)) {
        while (key != 'q') {
            ts = (double) cv::getTickCount();
            frames = camera.getFrame();

            std::vector<cv::Mat> channels;
            cv::Mat hsv;
            //cv::cvtColor(frames, hsv, COLOR_BGR2HSV);
            cv::split(frames, channels);

            cv::Mat gray;
            cv::inRange(channels[0], 220, 255, gray);

            rm::CalcGamma(frames, frames, 0.25);


            cv::imshow("aaa", channels[2] - channels[1]);
            cv::imshow("aaa2", frames);

            std::cout << 1 / (((double) cv::getTickCount() - ts) / cv::getTickFrequency()) << std::endl;
            key = cv::waitKey(1);
        }
    }

//    rm::SerialPort sp{};
//    sp.Initialize();
//
//    rm::Response response{};
//    response.yaw.data = 20.0;
//    response.pitch.data = 30.0;
//    response.rank = 3;
//
//    rm::Request request{};
//
//    thread t1([&]() {
//        while (true) {
//            std::cout << "send status: " << sp.Send(response) << std::endl;
//        }
//    });
//
//    thread t2([&]() {
//        while (true) {
//            if (sp.Receive(request)) {
//                std::cout << "received: ";
//                std::cout << "p: " << request.pitch.data << "  ";
//                std::cout << "camp: " << request.camp << "  ";
//                std::cout << "mode: " << request.mode << "  ";
//                std::cout << "speed: " << request.speed << std::endl;
//            }
//        }
//    });
//
//    t1.join();
//    t2.join();
//
//    return 0;
}
