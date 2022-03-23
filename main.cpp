#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

// TODO: everything use float, except PnP(double) (needs more investigation)

int main(int argc, char *argv[]) {
    // Debug mode
    if (argc == 2 && (*argv[1] == '1' || *argv[1] == '0')) {
        auto camp = *argv[1] == '0' ? rm::CAMP_RED : rm::CAMP_BLUE;
        rm::DahengCamera camera;
        bool cameraStatus = camera.dahengCameraInit((char *) "KE0210010003", 2500, 210);
        while (cameraStatus) {
            cv::Mat frame = camera.getFrame();
            if (frame.empty()) {
                cameraStatus = false;
            } else {
                cv::Mat binary;
                rm::ExtractColor(frame, binary, camp);
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

                // Fit armours
                std::vector<rm::LightBar> lightBars;
                std::vector<rm::Armour> armours;
                rm::FindLightBars(contours, lightBars, 2, 19, 90, 10);
                rm::FindArmour(lightBars, armours, 20, 8, 0.12, 0.5, 0.65, 0.24, {frame.cols, frame.rows}, camp);

                if (!armours.empty()) {
                    cv::Mat icon;
                    rm::CalcRatio(frame, icon, armours[0].icon, armours[0].iconBox, {30, 30});
                    rm::CalcGamma(icon, icon, 0.05);
                    std::vector<cv::Mat> channels;
                    cv::split(icon, channels);
                    cv::Mat gray = channels[*argv[1] == '1' ? 2 : 0] + channels[1];
                    gray.convertTo(gray, CV_32FC1);
                    cv::imshow("test", gray);

//                    cv::imwrite("/home/yaione/Desktop/sbdum2.jpg", icon);
                }

                rm::debug::DrawArmours(armours, frame, -1);
                rm::debug::DrawLightBars(lightBars, frame, -1);
                cv::imshow("frame", frame);
                cv::imshow("binary", binary);
                cv::waitKey(1);
            }
        }
    } else if (argc == 2 && *argv[1] == '3') {
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

        rm::DahengCamera camera;
        bool cameraStatus = camera.dahengCameraInit((char *) "KE0210010003", 2500, 210);
        Ptr<cv::ml::ANN_MLP> model = cv::ml::ANN_MLP::load("test2.xml");
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1279.7, 0, 619.4498, 0, 1279.1, 568.4985, 0, 0, 1);
        cv::Mat distCoeffs = (cv::Mat_<double>(1, 5)
                << -0.107365897147967, 0.353460341713276, 0, 0, -0.370048735508088);
        rm::Response response{0, 0, 0};
        int lastTarget = 1;
        long time = cv::getTickCount();
        while (cameraStatus) {
            cv::Mat frame = camera.getFrame();
            if (!frame.empty()) {
                // Extract color
                cv::Mat binary;
                rm::ExtractColor(frame, binary, static_cast<rm::CampType>(request.camp));

                // Find contours
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

                // Fit armours
                std::vector<rm::LightBar> lightBars;
                std::vector<rm::Armour> armours;
                rm::FindLightBars(contours, lightBars, 2, 19, 65, 10);
                rm::FindArmour(lightBars, armours, 20, 8, 0.12, 0.5, 0.65, 0.22, {frame.cols, frame.rows},
                               request.camp == 0 ? rm::CAMP_BLUE : rm::CAMP_RED);

                for (auto &armour: armours) {
                    cv::Mat icon;
                    rm::CalcRatio(frame, icon, armour.icon, armour.iconBox, {30, 30});
                    rm::CalcGamma(icon, icon, 0.05);
                    std::vector<cv::Mat> channels;
                    cv::split(icon, channels);
                    cv::Mat gray = channels[armour.campType == rm::CAMP_RED ? 0 : 2];
                    gray.convertTo(gray, CV_32FC1);
                    Mat cp;
                    cv::Mat rows = gray.reshape(0, 1);
                    model->predict(rows, cp);

                    double maxValue = 0;
                    int maxIndex[2] = {0};
                    cv::minMaxIdx(cp, nullptr, &maxValue, nullptr, maxIndex);
                    armour.forceType = maxValue > 0.9 ? static_cast<rm::ForceType>(maxIndex[1]) : rm::FORCE_UNKNOWN;
                }

                int index = -1;
                for (int i = 0; i < armours.size(); i++) {
                    if (armours[i].forceType == lastTarget) {
                        index = i;
                        continue;
                    } else if (armours[i].forceType != rm::FORCE_UNKNOWN && index < 0) {
                        index = i;
                        lastTarget = armours[i].forceType;
                    }
                }
                if (index != -1) {
                    rm::SolveArmourPose(armours[index], cameraMatrix, distCoeffs, {21.5, 5.5});
                    rm::SolveAirTrack(armours[index], 9.8, (int) request.speed, 10, request.pitch.data);
                    response.pitch.data = armours[index].pitch;
                    response.yaw.data = armours[index].yaw;
                    response.rank = armours[index].rank;
                    std::cout << (double) (cv::getTickCount() - time) / cv::getTickFrequency() << "  "
                              << response.pitch.data << "   " << armours[index].forceType << std::endl;
                    time = cv::getTickCount();

                    if (!serialPort.Send(response)) {
                        std::cout << response.pitch.data << std::endl;
                    }
                }
            }
        }
        serialPortThread.join();
    }

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
                rm::FindLightBars(contours, lightBars, 2, 19, 65, 10);
                rm::FindArmour(lightBars, result.armours, 20, 8, 0.12, 0.5, 0.65, 0.24,
                               {package->frame.cols, package->frame.rows}, package->camp);

                armourPackageQueue.push(result);

                cv::Mat temp;
                package->frame.copyTo(temp);
                rm::debug::DrawArmours(result.armours, temp, -1);
                rm::debug::DrawLightBars(lightBars, temp, -1);
                cv::imshow("test", temp);
                cv::imshow("test2", package->binary);
                cv::waitKey(1);
            }
        }
    });

    rm::ParallelQueue<rm::Package> targetQueue;
    Ptr<cv::ml::ANN_MLP> model = cv::ml::ANN_MLP::load("test2.xml");
    // MLP predicting thread
    thread MLPThread([&]() {
        while (frameStatus) {
            if (!targetQueue.Empty()) continue;
            auto package = armourPackageQueue.pop();
            if (package->armours.empty()) continue;

            rm::Package result(package);
            cv::parallel_for_(cv::Range(0, (int) package->armours.size()), [&](const cv::Range &range) {
                for (int i = range.start; i < range.end; i++) {
                    cv::Mat icon;
                    rm::CalcRatio(package->frame, icon, package->armours[i].icon, package->armours[i].iconBox,
                                  {30, 30});
                    rm::CalcGamma(icon, icon, 0.05);
                    std::vector<cv::Mat> channels;
                    cv::split(icon, channels);
                    cv::Mat gray = channels[package->camp == rm::CAMP_RED ? 0 : 2];
                    gray.convertTo(gray, CV_32FC1);
                    Mat cp;
                    cv::Mat rows = gray.reshape(0, 1);
                    model->predict(rows, cp);

                    double maxValue = 0;
                    int maxIndex[2] = {0};
                    cv::minMaxIdx(cp, nullptr, &maxValue, nullptr, maxIndex);

                    result.armours[i].forceType =
                            maxValue > 0.9 ? static_cast<rm::ForceType>(maxIndex[1]) : rm::FORCE_UNKNOWN;
                }
            }, cv::getNumThreads());
            targetQueue.push(result);
        }
    });

    rm::Response response{0, 0, 0};
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1279.7, 0, 619.4498, 0, 1279.1, 568.4985, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.107365897147967, 0.353460341713276, 0, 0, -0.370048735508088);
    thread solveThread([&]() {
        int lastTarget = 1;
        while (frameStatus) {
            auto package = targetQueue.pop();
            int index = -1;
            for (int i = 0; i < package->armours.size(); i++) {
                if (package->armours[i].forceType == lastTarget) {
                    index = i;
                    continue;
                } else if (package->armours[i].forceType != rm::FORCE_UNKNOWN && index < 0) {
                    index = i;
                    lastTarget = package->armours[i].forceType;
                }
            }
            if (index != -1) {
                rm::SolveArmourPose(package->armours[index], cameraMatrix, distCoeffs, {21.5, 5.5});
                rm::SolveAirTrack(package->armours[index], 9.8, package->speed, 10, package->pitch);
                response.pitch.data = package->armours[index].pitch;
                response.yaw.data = package->armours[index].yaw;
                response.rank = package->armours[index].rank;

                std::cout << package->armours[index].tvecs.ptr<double>(0)[2] << std::endl;

                if (!serialPort.Send(response)) {
                    std::cout << response.pitch.data << std::endl;
                }
            }
        }
    });

    serialPortThread.join();
    rawPackageThread.join();
    armourPackageThread.join();
    MLPThread.join();
    solveThread.join();

    return 0;
}
