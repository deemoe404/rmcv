#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

void print(Mat &mat, int prec) {
    for (int i = 0; i < mat.size().height; i++) {
        cout << "[ ";
        for (int j = 0; j < mat.size().width; j++) {
            cout << fixed << setw(2) << setprecision(prec) << mat.at<double>(i, j);
            if (j != mat.size().width - 1)
                cout << ", ";
            else
                cout << " ]" << endl;
        }
    }
}

void codeRotateByZ(double x, double y, double thetaz, double &outx, double &outy) {
    double x1 = x;
    double y1 = y;
    double rz = thetaz * CV_PI / 180;
    outx = cos(rz) * x1 - sin(rz) * y1;
    outy = sin(rz) * x1 + cos(rz) * y1;
}

void codeRotateByY(double x, double z, double thetay, double &outx, double &outz) {
    double x1 = x;
    double z1 = z;
    double ry = thetay * CV_PI / 180;
    outx = cos(ry) * x1 + sin(ry) * z1;
    outz = cos(ry) * z1 - sin(ry) * x1;
}

void codeRotateByX(double y, double z, double thetax, double &outy, double &outz) {
    double y1 = y;
    double z1 = z;
    double rx = thetax * CV_PI / 180;
    outy = cos(rx) * y1 - sin(rx) * z1;
    outz = cos(rx) * z1 + sin(rx) * y1;
}

int main() {
    rm::SerialPort serialPort;
    bool serialPortStatus = serialPort.Initialize();
    Ptr<cv::ml::ANN_MLP> model = cv::ml::ANN_MLP::load("test.xml");
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1279.7, 0, 619.4498, 0, 1279.1, 568.4985, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.107365897147967, 0.353460341713276, 0, 0, -0.370048735508088);

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

                    cv::Point2f exactSize(4.8, 12.3);
                    rm::SolveArmourPose(armour, cameraMatrix, distCoeffs, exactSize);

                    std::vector<std::vector<cv::Point>> tmp3;
                    std::vector<cv::Point> tmp2;
                    tmp2.push_back(armour.vertices[0]);
                    tmp2.push_back(armour.vertices[1]);
                    tmp2.push_back(armour.vertices[2]);
                    tmp2.push_back(armour.vertices[3]);
                    tmp3.push_back(tmp2);
                    cv::circle(package->frame, armour.vertices[2], 10, {0, 0, 255});
                    cv::drawContours(package->frame, tmp3, -1, {0, 255, 255}, 3);

                    cv::imshow("aaa", package->frame);
                    cv::waitKey(1);

                    cv::Mat test2;
                    cv::Rodrigues(armour.rvecs, test2);
                    double test = sqrt(pow(armour.tvecs.at<double>(0, 0), 2) + pow(armour.tvecs.at<double>(0, 1), 2) +
                                       pow(armour.tvecs.at<double>(0, 2), 2));

                    double thetaZ = atan2(test2.ptr<double>(1)[0], test2.ptr<double>(0)[0]) * 180 / CV_PI;
                    double thetaY = atan2(-test2.ptr<double>(2)[0],
                                          sqrt(pow(test2.ptr<double>(2)[0], 2) + pow(test2.ptr<double>(2)[2], 2))) *
                                    180 / CV_PI;
                    double thetaX = atan2(test2.ptr<double>(2)[1], test2.ptr<double>(2)[2]) * 180 / CV_PI;

                    double tx = armour.tvecs.ptr<double>(0)[0];
                    double ty = armour.tvecs.ptr<double>(0)[1];
                    double tz = armour.tvecs.ptr<double>(0)[2];
                    codeRotateByZ(tx, ty, -1 * thetaZ, tx, ty);
                    codeRotateByY(tx, tz, -1 * thetaY, tx, tz);
                    codeRotateByX(ty, tz, -1 * thetaX, ty, tz);
                    std::cout << test << std::endl;
//                    print(test2, 3);
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
