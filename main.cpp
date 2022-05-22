#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

// TODO: everything use float, except PnP(double) (needs more investigation)

struct Request {
    rm::CampType OwnCamp;
    rm::AimMode Mode;
    int FireRate;
    float GimbalPitch;
};

struct Response {
    float Pitch;
    float Yaw;
    unsigned char Rank;
};

int main(int argc, char *argv[]) {
    Request request{rm::CAMP_RED, rm::AIM_COMBAT, 0, 0};

    rm::SerialPort serialPort;
    bool status = serialPort.Initialize("/dev/ttyUSB0", B460800);
    thread readThread([&]() {
        while (status) {
            unsigned char readBuff[8];

            if (!serialPort.Receive(readBuff, 8)) {
                std::cout << "Serial port receive failed." << std::endl;
                continue;
            }

            if (readBuff[0] != 0x38) {
                std::cout << "Head error." << std::endl;
                continue;
            }

            if (readBuff[7] != rm::LookupCRC(readBuff, 7)) {
                std::cout << "CRC error." << std::endl;
                continue;
            }

            request.OwnCamp = static_cast<rm::CampType>(readBuff[1] & 0x01);
            request.Mode = (readBuff[1] & 0x04) >> 2 == '0' ? rm::AIM_COMBAT : rm::AIM_SNIPE;
            request.FireRate = (int) readBuff[2];

            std::memcpy(&request.GimbalPitch, readBuff + 3, sizeof(float));

            std::cout << request.GimbalPitch << std::endl;
        }
        std::cout << "Serial port read closed." << std::endl;
    });

    rm::ParallelQueue<Response> msgs;
    thread sendThread([&]() {
        shared_ptr<Response> msg;
        unsigned char sendBuff[11];
        while (status) {
            msg = msgs.pop();

            sendBuff[0] = 0x66;
            std::memcpy(sendBuff + 1, &msg->Pitch, sizeof(float));
            std::memcpy(sendBuff + 5, &msg->Yaw, sizeof(float));
            sendBuff[9] = msg->Rank;
            sendBuff[10] = rm::LookupCRC(sendBuff, 10);
            if (!serialPort.Send(sendBuff, 11)) {
                std::cout << "Send failed" << std::endl;
            }
        }
        std::cout << "Serial port send closed." << std::endl;
    });

    thread detectionThread([&]() {
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3)
                << 2893.316432895769, 0, 626.9759935670470, 0, 2932.394705738403, 523.2422551706520, 0, 0, 1);
        cv::Mat distCoeffs = (cv::Mat_<double>(1, 5)
                << -0.105453316958137, 0.369031456681063, 0.031896852060857, 0.002879995131883, -0.569188001455716);

        rm::ShootFactor result;
        cv::Mat source, calibration, binary, tvecs, rvecs;
        cv::Rect ROI;
        std::vector<rm::Contour> contours;
        std::vector<rm::LightBlob> lightBlobs(32);
        std::vector<rm::Armour> armours(16);

        rm::DahengCamera camera;
        bool status = camera.dahengCameraInit((char *) "KE0210010003", true, (int) (1.0 / 210.0 * 1000000), 0.0);
        while (status) {
            source = camera.getFrame(true);
            if (source.empty()) continue;
            if (ROI.width != 0 && ROI.height != 0) {
                calibration = source(ROI);
            } else {
                calibration = source;
            }

            if (request.Mode == rm::AIM_SNIPE) {
                rm::ExtractColor(calibration, binary, rm::CAMP_OUTPOST, 64);
                cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
                rm::FindLightBlobs(contours, lightBlobs, 1, 1.5, 360, 16, 2560, rm::CAMP_OUTPOST, true);

                if (!lightBlobs.empty()) {
                    rm::SolvePNP(lightBlobs[0].vertices, cameraMatrix, distCoeffs, {5.43, 5.43}, tvecs, rvecs);
                    rm::SolveShootFactor(tvecs, result, -0, 0, 0, {0, 0}, -0, rm::COMPENSATE_NONE);
                    std::cout << "  " << result.yawAngle << "  " << result.pitchAngle;
                    std::cout << rm::SolveDistance(tvecs) << "  " << tvecs.at<double>(2) << std::endl;
                }
                rm::debug::DrawlightBlobs(lightBlobs, calibration, -1);
                cv::imshow("frame", calibration);
                cv::imshow("binary", binary);
                cv::waitKey(1);
            } else if (request.Mode == rm::AIM_COMBAT) {
                rm::ExtractColor(calibration, binary, request.OwnCamp, 40, true, {5, 5});
                cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
                rm::FindLightBlobs(contours, lightBlobs, 2, 10, 25, 80, 1500, calibration, true);
                rm::FindArmour(lightBlobs, armours, 8, 24, 0.15, 0.45, 0.65, request.OwnCamp,
                               cv::Size2f{(float) calibration.cols, (float) calibration.rows});

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
                            ROI = rm::GetROI(armour.icon, 4, 1.5, {source.cols, source.rows}, ROI);
                            msgs.push({result.pitchAngle, result.yawAngle, 0});

                            rm::debug::DrawlightBlobs(lightBlobs, calibration, -1);
                            rm::debug::DrawArmours(armours, calibration, -1);
                            continue;
                        }
                    }
                }

                cv::imshow("frame", calibration);
                cv::imshow("binary", binary);
                cv::waitKey(1);
            } else if (request.Mode == rm::AIM_BUFF) {

            }
        }
    });

    readThread.join();
    sendThread.join();
    detectionThread.join();
    return 0;
}
