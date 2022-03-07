#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

int main() {
    DahengCamera camera;
    cv::Mat frames;
    if (camera.dahengCameraInit((char *) "KE0210030295", 2500, 5)) {
        while (1) {
            frames = camera.getFrame();
            cv::imshow("aaa", frames);
            cv::waitKey(1);
        }
    }

    return 0;
}
