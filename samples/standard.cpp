//
// Created by yaione on 22-5-27.
//

#include <iostream>
#include <thread>
#include "rmcv.h"

struct Request {
    rm::CampType Enemy;
    rm::AimMode Mode;
    int FireRate;
    float GimbalPitch;
    float GimbalYaw;
};

struct Response {
    float Pitch;
    float Yaw;
    unsigned char Rank; // dm
};

bool GetRequest(unsigned char *buffer, float fairAngle, Request &request) {
    if (buffer[0] != 0x38) {
        return false;
    }

    if (buffer[11] != rm::LookupCRC(buffer, 11)) {
        return false;
    }

    request.Enemy = static_cast<rm::CampType>(buffer[1] & 0x01) == rm::CAMP_BLUE ? rm::CAMP_RED : rm::CAMP_BLUE;
    request.Mode = (int) ((buffer[1] & 0x04) >> 2) == 0 ? rm::AIM_COMBAT : rm::AIM_BUFF;
    request.FireRate = (int) buffer[2];

    std::memcpy(&request.GimbalYaw, buffer + 3, sizeof(float));
    std::memcpy(&request.GimbalPitch, buffer + 7, sizeof(float));
    request.GimbalPitch = (float) ((fairAngle - request.GimbalPitch) / 8191.0f * CV_2PI);

    return true;
}

void FormatResponse(unsigned char *buffer, const shared_ptr<Response> &response) {
    buffer[0] = 0x66;
    std::memcpy(buffer + 1, &response->Pitch, sizeof(float));
    std::memcpy(buffer + 5, &response->Yaw, sizeof(float));
    buffer[9] = response->Rank;
    buffer[10] = rm::LookupCRC(buffer, 10);
};

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

        rm::ShootFactor result;
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
                rm::FindArmour(lightBlobs, armours, 8, 24, 0.15, 0.45, 0.65, request.Enemy,
                               cv::Size2f{(float) subImage.cols, (float) subImage.rows});

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
                    double h = rm::SolveDeltaHeight(tvecs, request.GimbalPitch, {0, 0}, 0);
                    double d = rm::SolveDistance(tvecs);
                    rm::SolveShootFactor(tvecs, result, -9.8, request.FireRate, h - 0, {0, 0}, rm::COMPENSATE_CLASSIC);
                    ROI = rm::GetROI(armours[0].icon, 4, {1.5, 1.15}, {image.cols, image.rows}, ROI);
                    message.push({result.pitchAngle, result.yawAngle, (unsigned char) (d / 10.0)});
                    std::cout << "p:" << result.pitchAngle << " y:" << result.yawAngle << " d:" << d << std::endl;
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
