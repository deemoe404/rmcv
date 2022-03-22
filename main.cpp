#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

// TODO: everything use float, except PnP(double) (needs more investigation)

int main(int argc, char *argv[]) {
    int count = 0;

    // Debug mode
    if (argc == 2) {
        rm::DahengCamera camera;
        bool cameraStatus = camera.dahengCameraInit((char *) "KE0210010003", 3500, 210);
        while (cameraStatus) {
            cv::Mat frame = camera.getFrame();
            if (frame.empty()) cameraStatus = false;
            else {
                cv::Mat binary;
                rm::ExtractColor(frame, binary, rm::CAMP_BLUE);
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

                // Fit armours
                std::vector<rm::LightBar> lightBars;
                std::vector<rm::Armour> armours;
                rm::FindLightBars(contours, lightBars, 2, 19, 20, 10);
                rm::FindArmour(lightBars, armours, 20, 15, 0.22, 0.5, 0.75, 0.24, {frame.cols, frame.rows},
                               rm::CAMP_BLUE);

                if (!armours.empty()) {
                    cv::Mat icon;
                    rm::CalcRatio(frame, icon, armours[0].icon, armours[0].iconBox, {30, 30});
                    rm::CalcGamma(icon, icon, 0.05);
//                    std::vector<cv::Mat> channels;
//                    cv::split(icon, channels);
//                    cv::Mat gray = channels[*argv[1] == '1' ? 2 : 0];
                    icon.convertTo(icon, CV_32FC1);

                    cv::imshow("test", icon);
                    rm::debug::DrawArmours(armours, frame, -1);

                }

                rm::debug::DrawLightBars(lightBars, frame, -1);
                cv::imshow("frame", frame);
                cv::imshow("binary", binary);
                cv::waitKey(1);
            }
        }
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
                rm::FindLightBars(contours, lightBars, 2, 19, 20, 10);
                rm::FindArmour(lightBars, result.armours, 20, 15, 0.22, 0.5, 0.75, 0.24,
                               {package->frame.cols, package->frame.rows}, package->camp);

                armourPackageQueue.push(result);

//                cv::Mat temp;
//                package->frame.copyTo(temp);
//                rm::debug::DrawArmours(result.armours, temp, -1);
//                rm::debug::DrawLightBars(lightBars, temp, -1);
//                cv::imshow("test", temp);
//                cv::waitKey(1);
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
                    cv::Mat gray = channels[2];
                    gray.convertTo(gray, CV_32FC1);
                    Mat cp;
                    cv::Mat rows = gray.reshape(0, 1);
                    model->predict(rows, cp);

                    int index = 0;
                    float highest = 0;
                    for (int j = 0; j < cp.cols; j++) {
                        if (cp.ptr<float>(0)[j] > highest) {
                            index = j;
                        }
                    }
                    result.armours[i].forceType =
                            cp.ptr<float>(0)[index] > 0 ? static_cast<rm::ForceType>(index) : rm::FORCE_UNKNOWN;
                    std::cout << result.armours[i].forceType << std::endl;
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
                rm::SolveArmourPose(package->armours[index], cameraMatrix, distCoeffs, {12.5, 5.5});
                rm::SolveAirTrack(package->armours[index], 9.8, package->speed, 10, package->pitch);
                response.pitch.data = package->armours[index].pitch;
                response.yaw.data = package->armours[index].yaw;
                response.rank = package->armours[index].rank;

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
