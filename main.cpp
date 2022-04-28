#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

// TODO: everything use float, except PnP(double) (needs more investigation)

class Request {
public:
    rm::CampType OwnCamp;
    rm::AimMode Mode;
    int FireRate;
    float GimbalPitch;

    Request() = default;

    Request(rm::CampType camp, rm::AimMode mode) : OwnCamp(camp), Mode(mode), FireRate(30), GimbalPitch(0) {}
};

class Response {
public:
    float Pitch;
    float Yaw;
    unsigned char Rank;

    Response() = default;

    Response(float pitch, float yaw, unsigned char rank) : Pitch(pitch), Yaw(yaw), Rank(rank) {};
};

int main(int argc, char *argv[]) {
    Request request(rm::CAMP_RED, rm::AIM_COMBAT);

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
            request.Mode = static_cast<rm::AimMode>((readBuff[1] & 0x04) >> 2);
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

    long tick = cv::getTickCount();
    thread detectionThread([&]() {
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1279.7, 0, 619.4498, 0, 1279.1, 568.4985, 0, 0, 1);
        cv::Mat distCoeffs = (cv::Mat_<double>(1, 5)
                << -0.107365897147967, 0.353460341713276, 0, 0, -0.370048735508088);

        rm::ShootFactor result;
        cv::Mat frame, binary, tvecs, rvecs;
        std::vector<rm::LightBar> lightBars(32);
        std::vector<rm::Armour> armours(16);

        rm::DahengCamera camera;
        bool status = camera.dahengCameraInit((char *) "KE0210030295", true, (int) (1.0 / 210.0 * 1000000), 0.0);
        while (status) {
            frame = camera.getFrame();
            tick = cv::getTickCount();
            if (!frame.empty()) {
                rm::ExtractColor(frame, binary, request.OwnCamp, true, 40, {5, 5});
                rm::FindLightBars(binary, lightBars, 2, 10, 25, 80, 1500, frame, true);
                rm::FindArmour(lightBars, armours, 8, 24, 0.15, 0.45, 0.65, request.OwnCamp,
                               cv::Size2f{(float) frame.cols, (float) frame.rows});

                for (auto &armour: armours) {
                    rm::SolvePNP(armour.vertices, cameraMatrix, distCoeffs, {5.5, 5.5}, tvecs, rvecs);
                    if (rm::SolveDistance(tvecs) < 450) {
                        rm::SolveDeltaHeight(tvecs, request.GimbalPitch);
                        rm::SolveShootFactor(tvecs, result, 9.8, request.FireRate, -60, {0, 0}, rm::COMPENSATE_CLASSIC);

                        std::cout << "pitch: " << result.pitchAngle << "  yaw: " << result.yawAngle << "  distance:"
                                  << rm::SolveDistance(tvecs) << "  ";
                        msgs.push({result.pitchAngle, result.yawAngle, 0});
                        continue;
                    }
                }
                std::cout << std::setiosflags(ios::fixed) << std::setiosflags(ios::right) << std::setprecision(6)
                          << armours.size() << ": "
                          << ((double) (cv::getTickCount() - tick) / cv::getTickFrequency()) * 1000.0 << "ms"
                          << std::endl;

                rm::debug::DrawLightBars(lightBars, frame, -1);
                rm::debug::DrawArmours(armours, frame, -1);
                cv::imshow("frame", frame);
                cv::waitKey(1);
            } else {
                status = false;
            }
        }
    });


    readThread.join();
    sendThread.join();
    detectionThread.join();
    return 0;
}
