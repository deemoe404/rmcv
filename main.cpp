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
            serialPort.Receive(request);
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
                rm::ExtractColor(frame, binary, rm::CAMP_BLUE);
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
                rm::Package armourPackage(package->camp, package->mode, request.speed, request.pitch.data,
                                          package->frame, package->binary);

                // Find contours
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(package->binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

                // Fit armours
                std::vector<rm::LightBar> lightBars;
                rm::FindLightBars(contours, lightBars, 3, 19, 20, 10);
                rm::FindArmour(lightBars, armourPackage.armours, 20, 10, 0.3, 0.7, 0.75,
                               {package->frame.cols, package->frame.rows});

                armourPackageQueue.push(armourPackage);
            }
        }
    });

    rm::ParallelQueue<rm::Armour> targetQueue;
    Ptr<cv::ml::ANN_MLP> model = cv::ml::ANN_MLP::load("test.xml");
    // MLP predicting thread
    thread MLPThread([&]() {
        while (frameStatus) {
            auto package = armourPackageQueue.pop();
            for (auto &armour: package->armours) {
                // TODO: Pack a class in objdetect, objdetect include mlp.h
                //       use parallel_for_ to predict all armour at the same time?!
                cv::Mat icon;
                rm::CalcRatio(package->frame, icon, armour.icon, armour.iconBox, {30, 30});
                rm::CalcGamma(icon, icon, 0.05);
                std::vector<cv::Mat> channels;
                cv::split(icon, channels);
                cv::Mat gray = channels[2];
                gray.convertTo(gray, CV_32FC1);
                Mat response;
                cv::Mat rows = gray.reshape(0, 1);
                model->predict(rows, response);

                // TODO: find the top one and push to
                if (response.at<float>(0, 1) > 0.9) { // TODO: !none->target! else remove at

                    targetQueue.push(armour);
                    continue;
                }
            }
        }
    });

    // TODO: this thread remember last targeting
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1279.7, 0, 619.4498, 0, 1279.1, 568.4985, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.107365897147967, 0.353460341713276, 0, 0, -0.370048735508088);
    rm::Response{0, 0, 0};
    // Pitch yaw solve thread
    thread solveThread([&]() {
        auto target = targetQueue.pop();
    });

    serialPortThread.join();
    rawPackageThread.join();
    armourPackageThread.join();
    MLPThread.join();
    solveThread.join();

    return 0;
}
