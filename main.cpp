#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

int main() {
    rm::SerialPort serialPort;
    bool serialPortStatus = serialPort.Initialize();
    Ptr<cv::ml::ANN_MLP> model = cv::ml::ANN_MLP::load("test.xml");
    cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3)
            << 227.5754394592698, 0, 961.2917903511311, 0, 224.9141431170980, 542.0294949020054, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<float>(1, 5)
            << -0.005276031771503, 0.0000274253772898775, 0, 0, 0.00000245864193872025);

    // Serial port request receiving thread
    rm::Request request{0, 0, 18, 0};
    thread serialPortThread([&]() {
        while (serialPortStatus) {
            serialPort.Receive(request);
        }
        std::cout << "Serial port closed." << std::endl;
    });

    // Frame capture thread
    rm::ParallelQueue<rm::Package> rawPackageQueue;
    rm::DahengCamera camera;
    bool cameraStatus = camera.dahengCameraInit((char *) "KE0210030295", 2500, 210);
    thread rawPackageThread([&]() {
        while (cameraStatus) {
            if (!rawPackageQueue.Empty()) continue;
            cv::Mat frame = camera.getFrame();
            if (!frame.empty()) {
                auto camp = static_cast<rm::CampType>(request.camp);
                auto mode = static_cast<rm::AimMode>(request.mode);
                cv::Mat binary;
                rm::ExtractColor(frame, binary, rm::CAMP_BLUE);
                rawPackageQueue.push(rm::Package(camp, mode, request.speed, request.pitch.data, frame, binary));
            }
        }
    });

    //
    rm::ParallelQueue<rm::Package> armourPackageQueue;
    bool frameStatus = true;
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

    thread solveThread([&]() {
        while (frameStatus) {
            auto package = armourPackageQueue.pop();
            for (auto &armour: package->armours) {
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
                if (response.at<float>(0, 1) > 0.9) {
                    cv::imshow("aaa", icon);
                    cv::waitKey(1);
                    cv::Point2f exactSize(56, 123.5);
                    rm::SolveArmourPose(armour, cameraMatrix, distCoeffs, exactSize);
                    std::cout << "0:" << armour.tvecs.at<double>(0) << "1:" << armour.tvecs.at<double>(1) << "2:"
                              << armour.tvecs.at<double>(2) << std::endl;
                    continue;
                }
            }
        }
    });

    serialPortThread.join();
    rawPackageThread.join();
    armourPackageThread.join();
    solveThread.join();

    return 0;
}
