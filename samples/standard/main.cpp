//
// Created by yaione on 22-5-27.
//

#include "main.h"

int main(int argc, char *argv[]) {
    Request request{rm::CAMP_BLUE, rm::AIM_COMBAT, 0, 0};

    // Info receiving thread
    rm::SerialPort serialPort;
    bool serialStatus = serialPort.Initialize("/dev/ttyUSB0", B460800);
    thread readThread([&]() {
        unsigned char readBuff[256];
        while (serialStatus) {
            if (!serialPort.Receive(readBuff, 12) || !GetRequest(readBuff, 4191, request)) {
                std::cout << "Receive failed." << std::endl;
            }
        }
        std::cout << "Serial port closed." << std::endl;
    });

    // result feedback thread
    rm::ParallelQueue<Response> message;
    thread sendThread([&]() {
        shared_ptr<Response> msg;
        unsigned char sendBuff[256];
        while (serialStatus) {
            FormatResponse(sendBuff, message.pop());
            if (!serialPort.Send(sendBuff, 11)) {
                std::cout << "Send failed." << std::endl;
            }
        }
    });

    // main detect thread
    thread detectionThread([&]() {
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3)
                << 897.869011646592, 0, 371.065213206808, 0, 897.752728356230, 276.689838317477, 0, 0, 1);
        cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.0818816085802673, 0.141075515191774, 0, 0, 0);

        cv::Mat subImage, image, binary, tvecs, rvecs;
        cv::Rect ROI;
        std::vector<rm::Contour> contours(32);
        std::vector<rm::LightBlob> lightBlobs(32);
        std::vector<rm::Armour> armours(16);

        rm::DahengCamera camera;
        bool status = camera.dahengCameraInit((char *) "FDK22050002", true, (int) (1.0 / 436.0 * 1000000), 0.0);

        long lostTick = -1;
        while (status) {
            if (request.Mode == rm::AIM_COMBAT) {
                image = camera.getFrame();
                if (image.empty()) continue;
                if (ROI.width != 0 && ROI.height != 0) {
                    subImage = image(ROI);
                } else {
                    subImage = image;
                }

                rm::ExtractColor(subImage, binary, request.Enemy, 80, false, {5, 5});
                cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
                rm::FindLightBlobs(contours, lightBlobs, 2, 10, 25, 80, 1500, subImage, true);
                rm::FindArmour(lightBlobs, armours, 8, 24, 0.15, 0.45, 0.65, request.Enemy);

                if (armours.empty()) {
                    if (lostTick == -1) {
                        lostTick = cv::getTickCount();
                    } else {
                        // If target lost, try to wait for the flash.
                        if ((double) (cv::getTickCount() - lostTick) / cv::getTickFrequency() > 0.05) {
                            ROI = cv::Rect(0, 0, 0, 0);
                            lostTick = -1;
                        }
                    }
                } else {
                    lostTick = -1;
                    rm::SolvePNP(armours[0].vertices, cameraMatrix, distCoeffs, {5.5, 5.5}, tvecs, rvecs, ROI);
                    double h = rm::DeltaHeight(tvecs, request.GimbalPitch, {0, 0}, 0);
                    double d = rm::SolveDistance(tvecs);
                }

                rm::debug::DrawLightBlobs(lightBlobs, image, -1);
                rm::debug::DrawArmours(armours, image, -1);
                cv::imshow("frame", image);
                cv::imshow("binary", binary);
                cv::waitKey(1);
            }
        }
        std::cout << "Camera closed." << std::endl;
    });

    readThread.join();
    sendThread.join();
    detectionThread.join();

    return 0;
}
