#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

// TODO: everything use float, except PnP(double) (needs more investigation)

int main() {
    rm::SerialPort serialPort;
    rm::Request request{0, 0, 18, 0};
    bool serialPortStatus = serialPort.Initialize();
    // Serial port request receiving thread
    thread serialPortThread([&]() {
        while (serialPortStatus) {
            if (!serialPort.Receive(request)) {
                std::cout << "Serial port receive failed." << std::endl;
            }
        }
        std::cout << "Serial port closed." << std::endl;
    });

    rm::ParallelQueue<rm::Package> rawPackageQueue;
    rm::DahengCamera camera;
    bool cameraStatus = camera.dahengCameraInit((char *) "KE0210010003", 2500, 210);
    // Frame capture thread
    thread rawPackageThread([&]() {
        while (cameraStatus) {
            if (!rawPackageQueue.Empty()) continue;
            cv::Mat frame = camera.getFrame();
            if (!frame.empty()) {
                // Extract color
                cv::Mat binary;
                rm::ExtractColor(frame, binary, static_cast<rm::CampType>(request.camp));
                rawPackageQueue.push(
                        rm::Package(static_cast<rm::CampType>(request.camp), static_cast<rm::AimMode>(request.mode),
                                    request.speed, request.pitch.data, frame, binary));
            }
        }
    });

    rm::ParallelQueue<rm::Package> armourPackageQueue;
    bool frameStatus = true;
    // Armour finding thread
    thread armourPackageThread([&]() {
        while (frameStatus) {
            if (!armourPackageQueue.Empty()) continue;
            auto package = rawPackageQueue.pop();

            if (!package->binary.empty()) {
                rm::Package result(package);

                // Find contours
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(package->binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

                // Fit armours
                std::vector<rm::LightBar> lightBars;
                rm::FindLightBars(contours, lightBars, 3, 19, 20, 10);
                rm::FindArmour(lightBars, result.armours, 20, 10, 0.3, 0.7, 0.75,
                               {package->frame.cols, package->frame.rows});

                armourPackageQueue.push(result);
            }
        }
    });

    rm::ParallelQueue<rm::Package> targetQueue;
    Ptr<cv::ml::ANN_MLP> model = cv::ml::ANN_MLP::load("test.xml");
    // MLP predicting thread
    thread MLPThread([&]() {
        while (frameStatus) {
            if (!targetQueue.Empty()) continue;
            auto package = armourPackageQueue.pop();
            rm::Package result(package);

            cv::parallel_for_(cv::Range(0, (int) package->armours.size()), [&](const cv::Range &range) {
                for (int i = range.start; i < range.end; i++) {
                    cv::Mat icon;
                    rm::CalcRatio(package->frame, icon, package->armours[i].icon, package->armours[i].iconBox,
                                  {30, 30});
                    rm::CalcGamma(icon, icon, 0.05);
                    std::vector<cv::Mat> channels;
                    cv::split(icon, channels);
                    cv::Mat gray = channels[2];
                    gray.convertTo(gray, CV_32FC1);
                    Mat cp;
                    cv::Mat rows = gray.reshape(0, 1);
                    model->predict(rows, cp);

                    int highest = 0;
                    for (; highest < cp.cols; highest++) {
                        if (cp.ptr<float>(0)[highest] > 0) {
                            break;
                        }
                    }
                    result.armours[i].forceType = static_cast<rm::ForceType>(highest);
                }
            }, cv::getNumberOfCPUs());
            targetQueue.push(result);
        }
    });

    // TODO: New thread
    //       go through package.armour, find the target closest to the frame center
    //       call solvePose & solveAir function to get p,y axis, then send to port
    rm::Response response{0, 0, 0};
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1279.7, 0, 619.4498, 0, 1279.1, 568.4985, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.107365897147967, 0.353460341713276, 0, 0, -0.370048735508088);

    serialPortThread.join();
    rawPackageThread.join();
    armourPackageThread.join();
    MLPThread.join();

    return 0;
}
