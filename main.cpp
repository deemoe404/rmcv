#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

int main() {
    DahengCamera camera;
    cv::Mat frames;
    auto ts = (double) cv::getTickCount();
    if (camera.dahengCameraInit((char *) "KE0210030295", 2500), 5) {
        while (1) {
            ts = (double) cv::getTickCount();
            frames = camera.getFrame();
            cv::imshow("aaa", frames);

            std::cout << 1 / ((cv::getTickCount() - ts) / cv::getTickFrequency()) << std::endl;
            cv::waitKey(1);
        }
    }

    return 0;
}
