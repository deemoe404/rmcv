#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

// TODO: everything use float, except PnP(double) (needs more investigation)

int main(int argc, char *argv[]) {
    rm::SerialPort serialPort;

    rm::Request request{0, 0, 18, 0};
    thread serialPortThread([&]() {
        bool status = serialPort.Initialize();
        int failure = 0;
        while (status) {
            if (!serialPort.Receive(request) && failure < 256) {
                std::cout << "Serial port receive failed." << std::endl;
                failure++;
            } else {
                status = false;
            }
        }
        std::cout << "Serial port closed." << std::endl;
    });

    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1279.7, 0, 619.4498, 0, 1279.1, 568.4985, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.107365897147967, 0.353460341713276, 0, 0, -0.370048735508088);
    thread detectionThread([&]() {
        rm::DahengCamera camera;
        bool status = camera.dahengCameraInit((char *) "KE0210030295", false, (int) (1.0 / 210.0 * 1000000), 0.0);
        while (status) {
            cv::Mat frame = camera.getFrame();
            auto ownCamp = static_cast<rm::CampType>(request.ownCamp);
            if (!frame.empty()) {
                cv::Mat binary;
                rm::ExtractColor(frame, binary, ownCamp, true, 40, {5, 5});

                std::vector<rm::LightBar> lightBars;
                rm::FindLightBars(binary, lightBars, 2, 10, 25, 80, 1500, frame, true);

                std::vector<rm::Armour> armours;
                rm::FindArmour(lightBars, armours, 8, 24, 0.15, 0.45, 0.65, ownCamp, {frame.cols, frame.rows});\

                for (auto &armour: armours) {
                    cv::Mat tvecs, rvecs;
                    rm::SolvePNP(armour.vertices, cameraMatrix, distCoeffs, {5.5, 5.5}, tvecs, rvecs);
                    double distance = rm::SolveDistance(tvecs);
                    std::cout << distance << std::endl;
                }

                rm::debug::DrawLightBars(lightBars, frame, -1);
                rm::debug::DrawArmours(armours, frame, -1);
                cv::imshow("frame", frame);
                cv::imshow("binary", binary);
                cv::waitKey(1);
            } else {
                status = false;
            }
        }
    });

    serialPortThread.join();
    detectionThread.join();
    return 0;
}
