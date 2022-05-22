#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

// TODO: everything use float, except PnP(double) (needs more investigation)

struct Request {
    rm::CampType OwnCamp;
    rm::AimMode Mode;
    int FireRate;
    float GimbalPitch;
    float GimbalYaw;
};

struct Response {
    float Pitch;
    float Yaw;
    unsigned char Rank;
};

bool GetRequest(unsigned char *buffer, int fairAngle, Request &request) {
    if (buffer[0] != 0x38) {
        return false;
    }

    if (buffer[11] != rm::LookupCRC(buffer, 11)) {
        return false;
    }

    request.OwnCamp = static_cast<rm::CampType>(buffer[1] & 0x01);
    request.Mode = (buffer[1] & 0x04) >> 2 == '0' ? rm::AIM_COMBAT : rm::AIM_SNIPE;
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
    rm::SerialPort serialPort;
    rm::ParallelQueue<Response> message;
    Request request{rm::CAMP_RED, rm::AIM_COMBAT, 0, 0};

    bool serialStatus = serialPort.Initialize("/dev/ttyUSB0", B460800);

    // Info receiving thread
    thread readThread([&]() {
        unsigned char readBuff[256];
        while (serialStatus) {
            if (!serialPort.Receive(readBuff, 12) || !GetRequest(readBuff, 4191, request)) {
                std::cout << "Serial port receive failed." << std::endl;
            }
        }
        std::cout << "Serial port read closed." << std::endl;
    });

    // result feedback thread
    thread sendThread([&]() {
        shared_ptr<Response> msg;
        unsigned char sendBuff[256];
        while (serialStatus) {
            FormatResponse(sendBuff, message.pop());
            if (!serialPort.Send(sendBuff, 11)) {
                std::cout << "Send failed" << std::endl;
            }
        }
        std::cout << "Serial port send closed." << std::endl;
    });

    // main detect thread
    thread detectionThread([&]() {
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3)
                << 2893.316432895769, 0, 626.9759935670470, 0, 2932.394705738403, 523.2422551706520, 0, 0, 1);
        cv::Mat distCoeffs = (cv::Mat_<double>(1, 5)
                << -0.105453316958137, 0.369031456681063, 0.031896852060857, 0.002879995131883, -0.569188001455716);

        rm::ShootFactor result;
        cv::Mat subImage, source0, source1, binary, tvecs, rvecs;
        cv::Rect ROI;
        std::vector<rm::Contour> contours;
        std::vector<rm::LightBlob> lightBlobs(32);
        std::vector<rm::Armour> armours(16);

        rm::DahengCamera camera0, camera1;
        bool camStatus0 = camera0.dahengCameraInit((char *) "KE0210010003", true, (int) (1.0 / 210.0 * 1000000), 0.0);
        bool camStatus1 = camera1.dahengCameraInit((char *) "KE0210010029", true, (int) (800), 0.0);
        while (true) {
            // snipe mode
            if (request.Mode == rm::AIM_SNIPE && camStatus1) {
                source1 = camera1.getFrame(false);
                if (source1.empty()) continue;
                if (ROI.width != 0 && ROI.height != 0) {
                    subImage = source1(ROI);
                } else {
                    subImage = source1;
                }

                rm::ExtractColor(subImage, binary, rm::CAMP_OUTPOST, 64);
                cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
                rm::FindLightBlobs(contours, lightBlobs, 1, 1.5, 360, 16, 2560, rm::CAMP_OUTPOST, true);
                if (!lightBlobs.empty()) {
                    rm::SolvePNP(lightBlobs[0].vertices, cameraMatrix, distCoeffs, {5.43, 5.43}, tvecs, rvecs, ROI);
                    rm::SolveShootFactor(tvecs, result, -0, 0, 0, {0, 0}, -0, rm::COMPENSATE_NONE);
                    std::cout << "  " << result.yawAngle << "  " << result.pitchAngle;
                    std::cout << rm::SolveDistance(tvecs) << "  " << tvecs.at<double>(2) << std::endl;
                }
                rm::debug::DrawlightBlobs(lightBlobs, subImage, -1);
                cv::imshow("frame", subImage);
                cv::imshow("binary", binary);
                cv::waitKey(1);
            }

            // combat mode
            if (request.Mode == rm::AIM_COMBAT && camStatus0) {
                source0 = camera0.getFrame(false);
                if (source0.empty()) continue;
                if (ROI.width != 0 && ROI.height != 0) {
                    subImage = source0(ROI);
                } else {
                    subImage = source0;
                }

                rm::ExtractColor(subImage, binary, request.OwnCamp, 40, true, {5, 5});
                cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
                rm::FindLightBlobs(contours, lightBlobs, 2, 10, 25, 80, 1500, subImage, true);
                rm::FindArmour(lightBlobs, armours, 8, 24, 0.15, 0.45, 0.65, request.OwnCamp,
                               cv::Size2f{(float) subImage.cols, (float) subImage.rows});
                if (armours.empty()) {
                    ROI = cv::Rect(0, 0, 0, 0);
                } else {
                    for (auto &armour: armours) {
                        rm::SolvePNP(armour.vertices, cameraMatrix, distCoeffs, {5.5, 5.5}, tvecs, rvecs, ROI);
                        if (rm::SolveDistance(tvecs) < 900) {
                            rm::SolveDeltaHeight(tvecs, request.GimbalPitch);
                            rm::SolveShootFactor(tvecs, result, -9.8, request.FireRate, -60, {0, 0},
                                                 rm::COMPENSATE_CLASSIC);

                            std::cout << "pitch: " << result.pitchAngle << "  yaw: " << result.yawAngle << "  distance:"
                                      << rm::SolveDistance(tvecs) << std::endl;
                            ROI = rm::GetROI(armour.icon, 4, 1.5, {source0.cols, source0.rows}, ROI);
                            message.push({result.pitchAngle, result.yawAngle, 0});

                            rm::debug::DrawlightBlobs(lightBlobs, subImage, -1);
                            rm::debug::DrawArmours(armours, subImage, -1);
                            continue;
                        }
                    }
                }

                cv::imshow("frame", subImage);
                cv::imshow("binary", binary);
                cv::waitKey(1);
            }
            if (request.Mode == rm::AIM_BUFF) {

            }
        }
    });

    readThread.join();
    sendThread.join();
    detectionThread.join();
    return 0;
}
