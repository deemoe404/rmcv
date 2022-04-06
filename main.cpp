#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

// TODO: everything use float, except PnP(double) (needs more investigation)

std::vector<cv::Mat> dataset;

void testRead() {
    cv::FileStorage fsread("/home/yaione/Desktop/0.xml", cv::FileStorage::READ);
    cv::Mat test;// = cv::imread("/home/yaione/Desktop/5.jpg");
//    test.convertTo(test, CV_32FC1);
    fsread["Mat4000"] >> test;
    cv::imshow("aa", test);
    cv::waitKey(0);
}

void debugMode(char input) {
    cv::FileStorage fs("/home/yaione/Desktop/0.xml", cv::FileStorage::WRITE);
    auto camp = input == '0' ? rm::CAMP_RED : rm::CAMP_BLUE;
    int count = 0;
    rm::DahengCamera camera;
    bool cameraStatus = camera.dahengCameraInit((char *) "KE0210030295", 2500, 210);
    int inputnum = 0;
    while (inputnum != 'q') {
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
            rm::FindLightBars(contours, lightBars, 2, 19, 65, 10);
            rm::FindArmour(lightBars, armours, 20, 8, 0.12, 0.5, 0.65, {frame.cols, frame.rows}, camp);

            if (!armours.empty()) {
                for (auto &item: armours) {
                    cv::Mat icon;
                    rm::CalcRatio(frame, icon, item.icon, item.iconBox, {28, 28});
                    cv::cvtColor(icon, icon, COLOR_BGR2GRAY);
                    icon.convertTo(icon, CV_32FC1);
                    fs << "Mat" + rm::int2str(count) << icon;
                    cv::imshow("icon", icon);
                    count++;
                    std::cout << count << std::endl;
                }
            }

            rm::debug::DrawArmours(armours, frame, -1);
            rm::debug::DrawLightBars(lightBars, frame, -1);
            cv::resize(frame, frame, {800, 600});
            cv::imshow("frame", frame);
//            cv::imshow("binary", binary);
            inputnum = cv::waitKey(1);
        }
    }
    fs.release();
}

int main(int argc, char *argv[]) {
    /// Debug mode
    if (argc == 2 && (*argv[1] == '1' || *argv[1] == '0')) {
//        testRead();
        debugMode(*argv[1]);
        return 0;
    }

    if (argc == 2 && *argv[1] == '3') {
        rm::DahengCamera camera;
        long ts = cv::getTickCount();
        bool cameraStatus = camera.dahengCameraInit((char *) "KE0210030295", (int) (1.0 / 210.0), 210);
        while (cameraStatus) {
            cv::Mat frame = camera.getFrame();
            if (!frame.empty()) {
//                std::cout << 1 / ((double) (cv::getTickCount() - ts) / cv::getTickFrequency()) << std::endl;
//                ts = cv::getTickCount();
//                rm::CalcGamma(frame, frame, 0.7);

                cv::Mat binary;
//                cv::inRange(frame, cv::Scalar{250, 80, 80}, cv::Scalar{255, 255, 255}, binary);
                cv::inRange(frame, cv::Scalar{0, 0, 250}, cv::Scalar{255, 255, 255}, binary);
                auto kernel = cv::getStructuringElement(cv::MORPH_RECT, {5, 5});
                cv::morphologyEx(binary, binary, cv::MORPH_DILATE, kernel);

                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
                std::vector<rm::LightBar> lightBars;
                rm::FindLightBars(contours, lightBars, 2, 19, 65, 10, frame);

                std::vector<rm::Armour> armours;
                if (!lightBars.empty()) {
                    rm::FindArmour(lightBars, armours, 20, 8, 0.12, 0.5, 0.65, {frame.cols, frame.rows},
                                   lightBars[0].camp);
                }

                if (!armours.empty()) {
                    cv::Mat icon;
                    rm::CalcRatio(frame, icon, armours[0].icon, armours[0].iconBox, {28, 28});
//                    icon.convertTo(icon, CV_32FC1);
                    cv::cvtColor(icon, icon, cv::COLOR_BGR2GRAY);
                    icon.convertTo(icon, CV_32FC1, 1.0 / 255.0);
                    std::cout << (float) icon.at<float>(0, 14) << std::endl;
                    cv::imshow("icon", icon);
                }

                rm::debug::DrawLightBars(lightBars, frame, -1);
                rm::debug::DrawArmours(armours, frame, -1);
                cv::imshow("frame", frame);
                cv::imshow("binary", binary);

                cv::waitKey(1);
            }
        }
    }

    /// Concurrent mode
    rm::SerialPort serialPort;
    rm::Request request{0, 0, 18, 0};
    // Serial port request receiving thread
    thread serialPortThread([&]() {
        bool status = serialPort.Initialize();
        while (status) {
            if (!serialPort.Receive(request)) {
                std::cout << "Serial port receive failed." << std::endl;
            }
        }
        std::cout << "Serial port closed." << std::endl;
    });

    rm::ParallelQueue<rm::Package> rawPackageQueue;
    rm::DahengCamera camera;
    bool cameraStatus = camera.dahengCameraInit((char *) "KE0210030295", 2500, 210);
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
                rm::FindArmour(lightBars, result.armours, 20, 8, 0.12, 0.5, 0.65,
                               {package->frame.cols, package->frame.rows}, package->camp);

                armourPackageQueue.push(result);

//                cv::Mat temp;
//                package->frame.copyTo(temp);
//                rm::debug::DrawArmours(result.armours, temp, -1);
//                rm::debug::DrawLightBars(lightBars, temp, -1);
//                cv::imshow("test", temp);
//                cv::imshow("test2", package->binary);
//                cv::waitKey(1);
            }
        }
    });

    rm::ParallelQueue<rm::Package> targetQueue;
    Ptr<cv::ml::ANN_MLP> model = cv::ml::ANN_MLP::load("/home/yaione/Desktop/test.xml");
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1279.7, 0, 619.4498, 0, 1279.1, 568.4985, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.107365897147967, 0.353460341713276, 0, 0, -0.370048735508088);
    // MLP predicting thread
    thread MLPThread([&]() {
        while (frameStatus) {
            if (!targetQueue.Empty()) continue;
            auto package = armourPackageQueue.pop();
            if (package->armours.empty()) continue;

            cv::Mat test;

            rm::Package result(package);
//            cv::parallel_for_(cv::Range(0, (int) package->armours.size()), [&](const cv::Range &range) {
//                for (int i = range.start; i < range.end; i++) {
            for (int i = 0; i < package->armours.size(); i++) {
                cv::Mat icon;
                rm::CalcRatio(package->frame, icon, package->armours[i].icon, package->armours[i].iconBox, {28, 28});
                cv::cvtColor(icon, icon, COLOR_BGR2GRAY);
//                auto kernel = cv::getStructuringElement(cv::MORPH_RECT, {2, 2});
//                cv::erode(icon, icon, kernel, {-1, -1}, 1);
//                cv::dilate(icon, icon, kernel, {-1, -1}, 1);
//                cv::threshold(icon, icon, 0, 255, cv::THRESH_BINARY);
//                std::vector<std::vector<cv::Point>> ctws;
//                cv::findContours(icon, ctws, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
//                int maxIndex = 0;
//                double maxValue = 0;
//                for (int j = 0; j < ctws.size(); j++) {
//                    if (cv::contourArea(ctws[j]) > maxValue) {
//                        maxValue = cv::contourArea(ctws[j]);
//                        maxIndex = j;
//                    }
//                }
//                for (int j = 0; j < ctws.size(); j++) {
//                    if (j != maxIndex) {
//                        cv::rectangle(icon, cv::boundingRect(ctws[j]), {0}, -1);
//                    }
//                }
                icon.convertTo(icon, CV_32FC1);
                Mat cp;
                cv::Mat rows = icon.reshape(0, 1);
                model->predict(rows, cp);

                if (i == package->armours.size() - 1) icon.copyTo(test);

                double maxValue2 = 0;
                int maxIndex2[2] = {0};
                cv::minMaxIdx(cp, nullptr, &maxValue2, nullptr, maxIndex2);

                result.armours[i].forceType =
                        maxValue2 > 0.99 ? static_cast<rm::ForceType>(maxIndex2[1]) : rm::FORCE_UNKNOWN;
//                if (maxValue2 > 0.9 && maxIndex2[1] != 0) {
                std::cout << rm::int2str(i);
                print(cp);
                std::cout << maxValue2 << std::endl;
//                }


                // ignore if distance farther than around 3m
                if (result.armours[i].forceType != rm::FORCE_UNKNOWN) {
                    rm::SolveArmourPose(result.armours[i], cameraMatrix, distCoeffs, cv::Size2f{5.5, 5.5});
                    if (result.armours[i].tvecs.ptr<double>(0)[2] > 350) {
//                        result.armours[i].forceType = rm::FORCE_UNKNOWN;
                    }
                }
            }
//            }, cv::getNumberOfCPUs());
            targetQueue.push(result);

            cv::Mat temp;
            package->frame.copyTo(temp);

            for (int i = 0; i < result.armours.size(); i++) {
                if (result.armours[i].forceType == rm::FORCE_UNKNOWN) continue;
                rm::debug::DrawArmours(result.armours, temp, i);
            }

            cv::imshow("test", temp);
            cv::imshow("test222", test);
            cv::imshow("test2", package->binary);
            cv::waitKey(1);
        }
    });

    rm::Response response{0, 0, 0};
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
                rm::SolveAirTrack(package->armours[index], 9.8, package->speed, 10, package->pitch);
                response.pitch.data = package->armours[index].pitch;
                response.yaw.data = package->armours[index].yaw;
                response.rank = package->armours[index].rank;

//                std::cout << package->armours[index].forceType << "  "
//                          << package->armours[index].tvecs.ptr<double>(0)[2] << std::endl;

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
