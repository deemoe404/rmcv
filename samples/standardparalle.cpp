//
// Created by yaione on 22-5-27.
//

#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

struct Request {
    rm::CampType Enemy;
    rm::AimMode Mode;
    int FireRate;
    float GimbalPitch;
    float GimbalYaw;
    float DeltaYaw;
    double DeltaTime;

    cv::Mat Frame;
};

struct Response {
    float Pitch;
    float Yaw;
    unsigned char Rank; // dm
};

bool GetRequest(unsigned char *buffer, int fairAngle, Request &request) {
    if (buffer[0] != 0x38) {
        return false;
    }

    if (buffer[11] != rm::LookupCRC(buffer, 11)) {
        return false;
    }

    request.Enemy = static_cast<rm::CampType>(buffer[1] & 0x01) == rm::CAMP_BLUE ? rm::CAMP_RED : rm::CAMP_BLUE;
    request.Mode = (int) ((buffer[1] & 0x04) >> 2) == 0 ? rm::AIM_COMBAT : rm::AIM_SNIPE;
    request.FireRate = (int) buffer[2];

    std::memcpy(&request.GimbalYaw, buffer + 3, sizeof(float));
    std::memcpy(&request.GimbalPitch, buffer + 7, sizeof(float));
    request.GimbalPitch = (((float) fairAngle - request.GimbalPitch) / 8191.0f) * 360.0f;

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
    cv::Mat a = cv::imread("/home/yaione/Downloads/123361-Made-in-Abyss-furry-anime (1).jpg");
    rm::ParallelQueue<Request> test;
    std::cout << sizeof(test) << std::endl;
    thread at([&]() {
        while (1) {
            if (test.Empty()) {
                test.push({rm::CAMP_BLUE, rm::AIM_COMBAT, 0, 0, 0, 0, 0, a});
                std::cout << "pushed!" << std::endl;
            } else {
                continue;
            }
        }
    });
    thread bt([&]() {
        while (1) {
            auto abc = test.tryPop();
            if (abc != nullptr) {
                std::cout << "poped!" << std::endl;
                if (abc->Frame.empty()) {
                    std::cout << "8888888888888888888888888888888888888888888888888888888888888888888888888888888888"
                              << std::endl;
                }
            }
        }
    });
    at.join();
    bt.join();

    std::cout << sizeof(test) << std::endl;


    // Info receiving thread
    rm::SerialPort serialPort;
    rm::ParallelQueue<Request> requestQueue;
    bool serialStatus = serialPort.Initialize("/dev/ttyUSB0", B460800);
    thread readThread([&]() {
        rm::DahengCamera camera;
        bool cameraStatus = camera.dahengCameraInit((char *) "FDK22050002", true, (int) (1.0 / 436.0 * 1000000), 0.0);

        unsigned char readBuff[256];
        float latestYaw = 0;
        long latestTick = 0, tick = 0;
        while (serialStatus && cameraStatus) {
            Request request{rm::CAMP_BLUE, rm::AIM_COMBAT, 0, 0};
            if (!serialPort.Receive(readBuff, 12) || !GetRequest(readBuff, 4191, request)) {
                std::cout << "Receive failed." << std::endl;
                continue;
            }

            request.Frame = camera.getFrame();
            if (request.Frame.empty()) {
                std::cout << "Camera closed." << std::endl;
                break;
            }

            if (!requestQueue.Empty()) {
                requestQueue.pop();
            }

            tick = cv::getTickCount();
            request.DeltaTime = ((double) (tick - latestTick)) / cv::getTickFrequency();
            request.DeltaYaw = request.GimbalYaw - latestYaw;

            latestYaw = request.GimbalYaw;
            latestTick = tick;

            requestQueue.push(request);
        }
        std::cout << "Serial port closed." << std::endl;
        std::cout << "Camera closed." << std::endl;
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
        std::vector<rm::Contour> contours;
        std::vector<rm::LightBlob> lightBlobs(32);
        std::vector<rm::Armour> armours(16);
        double latestDeltaYaw = 0;

        while (true) {
            auto request = requestQueue.pop();
            if (request->Mode == rm::AIM_COMBAT) {
                if (ROI.width != 0 && ROI.height != 0) {
                    subImage = image(ROI);
                } else {
                    subImage = image;
                }

                rm::ExtractColor(subImage, binary, request->Enemy, 80, false, {5, 5});
                cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
                rm::FindLightBlobs(contours, lightBlobs, 2, 10, 25, 80, 1500, subImage, true);
                rm::FindArmour(lightBlobs, armours, 8, 24, 0.15, 0.45, 0.65, request->Enemy,
                               cv::Size2f{(float) subImage.cols, (float) subImage.rows});
                if (armours.empty()) {
                    ROI = cv::Rect(0, 0, 0, 0);
                } else {
                    for (auto &armour: armours) {
                        rm::SolvePNP(armour.vertices, cameraMatrix, distCoeffs, {5.5, 5.5}, tvecs, rvecs, ROI);
                        double h = rm::SolveDeltaHeight(tvecs, request->GimbalYaw, {0, 0}, 0);
                        double d = rm::SolveDistance(tvecs);
                        rm::SolveShootFactor(tvecs, result, -9.8, request->GimbalYaw, h - 0, {0, 0},
                                             rm::COMPENSATE_CLASSIC);
                        ROI = rm::GetROI(armour.icon, 4, 1.15, {image.cols, image.rows}, ROI);
                        message.push({result.pitchAngle, result.yawAngle, static_cast<unsigned char>((d / 10.0))});
                        latestDeltaYaw = result.yawAngle;
                        std::cout << "p:" << result.pitchAngle << " y:" << result.yawAngle << " d:" << d << std::endl;
                        break;
                    }
                }

                rm::debug::DrawLightBlobs(lightBlobs, image, -1);
                rm::debug::DrawArmours(armours, image, -1);
                cv::imshow("frame", image);
                cv::imshow("binary", binary);
                cv::waitKey(1);
            }
        }
    });

    readThread.join();
    sendThread.join();
    detectionThread.join();

    return 0;
}
