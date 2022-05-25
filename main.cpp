#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"
#include "openvino/openvino.hpp"

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
    unsigned char Rank; // dm
};

struct RecognizeBox {
    cv::Point2f vertices[2];
    float score;
    int label;
};

bool GetRequest(unsigned char *buffer, int fairAngle, Request &request) {
    if (buffer[0] != 0x38) {
        return false;
    }

    if (buffer[11] != rm::LookupCRC(buffer, 11)) {
        return false;
    }

    request.OwnCamp = static_cast<rm::CampType>(buffer[1] & 0x01);
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

void NMS(std::vector<RecognizeBox> &input, float thresh) {
    std::sort(input.begin(), input.end(), [](RecognizeBox box0, RecognizeBox box1) { return box0.score > box1.score; });
    std::vector<float> vArea(input.size());
    for (int i = 0; i < int(input.size()); i++) {
        vArea[i] = (input[i].vertices[1].x - input[i].vertices[0].x + 1) *
                   (input[i].vertices[1].y - input[i].vertices[0].y + 1);
    }
    for (int i = 0; i < int(input.size()); i++) {
        for (int j = i + 1; j < int(input.size());) {
            float w = std::max(float(0), std::min(input[i].vertices[1].x, input[j].vertices[1].x) -
                                         std::max(input[i].vertices[0].x, input[j].vertices[0].x) + 1);
            float h = std::max(float(0), std::min(input[i].vertices[1].y, input[j].vertices[1].y) -
                                         std::max(input[i].vertices[0].y, input[j].vertices[0].y) + 1);
            float inter = w * h;
            float ovr = inter / (vArea[i] + vArea[j] - inter);
            if (ovr >= thresh) {
                input.erase(input.begin() + j);
                vArea.erase(vArea.begin() + j);
            } else {
                j++;
            }
        }
    }
}

std::vector<RecognizeBox>
DecodeInfer(const ov::Tensor &boxTensor, const ov::Tensor &scoreTensor, float threshold, float scaleX, float scaleY,
            int classCount) {
    std::vector<RecognizeBox> results;
    auto scoreShape = scoreTensor.get_shape()[2];
    const auto *boxData = boxTensor.data<const float>();
    const auto *scoreData = scoreTensor.data<const float>();
    for (size_t i = 0; i < classCount; i++) {
        for (size_t j = scoreShape * i; j < scoreShape * (i + 1); j++) {
            if (scoreData[j] > threshold) {
                int index = (int) (j - scoreShape * i) * 4;
                results.push_back(
                        {{{boxData[index], boxData[index + 1]}, {boxData[index + 2], boxData[index + 3]}}, scoreData[j],
                         (int) i});
            }
        }
    }
    NMS(results, 0.5);
    for (auto &result: results) {
        result.vertices[0].x = result.vertices[0].x * scaleX;
        result.vertices[0].y = result.vertices[0].y * scaleY;
        result.vertices[1].x = result.vertices[1].x * scaleX;
        result.vertices[1].y = result.vertices[1].y * scaleY;
    }
    return results;
}

int main(int argc, char *argv[]) {
    ov::Core core;
    ov::Shape input_shape = {1, 3, 270, 320};
    std::shared_ptr<ov::Model> model = core.read_model("picodet5_12b.xml");
    ov::CompiledModel compiled_model = core.compile_model(model, "CPU");
    ov::InferRequest infer_request = compiled_model.create_infer_request();
    auto input_port = compiled_model.input();

    rm::SerialPort serialPort;
    rm::ParallelQueue<Response> message;
    Request request{rm::CAMP_RED, rm::AIM_COMBAT, 0, 0};

    bool serialStatus = serialPort.Initialize("/dev/ttyUSB0", B460800);

    // Info receiving thread
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
    thread sendThread([&]() {
        shared_ptr<Response> msg;
        unsigned char sendBuff[256];
        while (serialStatus) {
            FormatResponse(sendBuff, message.pop());
            if (!serialPort.Send(sendBuff, 11)) {
                std::cout << "Send failed" << std::endl;
            }
        }
    });

    // main detect thread
    thread detectionThread([&]() {
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3)
                << 2893.316432895769, 0, 626.9759935670470, 0, 2932.394705738403, 523.2422551706520, 0, 0, 1);
        cv::Mat distCoeffs = (cv::Mat_<double>(1, 5)
                << -0.105453316958137, 0.369031456681063, 0.031896852060857, 0.002879995131883, -0.569188001455716);

        rm::ShootFactor result;
        cv::Mat subImage, mainCam, sideCam, binary, tvecs, rvecs;
        cv::Rect ROI;
        std::vector<rm::Contour> contours;
        std::vector<rm::LightBlob> lightBlobs(32);
        std::vector<rm::Armour> armours(16);

        rm::DahengCamera camera0, camera1;
        bool camStatus0 = camera0.dahengCameraInit((char *) "KE0210010004", true, (int) (1.0 / 210.0 * 1000000), 1.0);
        bool camStatus1 = camera1.dahengCameraInit((char *) "KE0210010029", true, (int) (800), 0.0);
        while (true) {
            // snipe mode
            if (request.Mode == rm::AIM_SNIPE && camStatus1) {
                sideCam = camera1.getFrame(false);
                if (sideCam.empty()) continue;
                if (ROI.width != 0 && ROI.height != 0) {
                    subImage = sideCam(ROI);
                } else {
                    subImage = sideCam;
                }

                rm::ExtractColor(subImage, binary, rm::CAMP_GUIDELIGHT, 64);
                cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
                rm::FindLightBlobs(contours, lightBlobs, 1, 1.5, 360, 16, 2560, sideCam, true);
                if (!lightBlobs.empty()) {
                    for (auto &lightBlob: lightBlobs) {
                        if (lightBlob.camp == rm::CAMP_GUIDELIGHT) {
                            rm::SolvePNP(lightBlob.vertices, cameraMatrix, distCoeffs, {5.43, 5.43}, tvecs, rvecs, ROI);
                            rm::SolveShootFactor(tvecs, result, -0, 0, 0, {0, 0}, -0, rm::COMPENSATE_NONE);
                            std::cout << result.yawAngle << "  " << result.pitchAngle << "  "
                                      << rm::SolveDistance(tvecs) << "  " << tvecs.at<double>(2) << std::endl;
                            break;
                        }
                    }
                    rm::debug::DrawLightBlobs(lightBlobs, subImage, -1);
                }

                cv::imshow("frame", subImage);
                cv::imshow("binary", binary);
                cv::waitKey(1);
            }

            // combat mode
            if (request.Mode == rm::AIM_COMBAT && camStatus0) {
                mainCam = camera0.getFrame(true, true);
                if (mainCam.empty()) continue;

                if (1) {//ROI.width == 0 && ROI.height == 0
                    auto blob = cv::dnn::blobFromImage(mainCam, 1.0 / 255.0, {320, 270}, true, false, CV_32F);
                    infer_request.set_input_tensor({input_port.get_element_type(), input_shape, (float *) blob.data});
                    infer_request.infer();

                    auto boxes = DecodeInfer(infer_request.get_output_tensor(0), infer_request.get_output_tensor(1),
                                             0.5f, (float) mainCam.cols / 320.0f, (float) mainCam.rows / 320.0f, 4);
                    if (!boxes.empty()) {
                        std::sort(boxes.begin(), boxes.end(), [&](RecognizeBox box0, RecognizeBox box1) {
                            return rm::PointDistance(rm::LineCenter(box0.vertices[0], box0.vertices[1]),
                                                     {(float) mainCam.cols / 2.0f, (float) mainCam.rows / 2.0f}) <
                                   rm::PointDistance(rm::LineCenter(box1.vertices[0], box1.vertices[1]),
                                                     {(float) mainCam.cols / 2.0f, (float) mainCam.rows / 2.0f});
                        });
                        for (auto &box: boxes) {
                            // cv::rectangle(mainCam, cv::Rect(box.vertices[0], box.vertices[1]), Scalar(0, 255, 0));
                            if (box.score > 0.5) {
                                ROI = rm::GetROI(box.vertices, 2, 1.5,
                                                 {(int) camera0.SensorWidth, (int) camera0.SensorHeight});;
                                break;
                            }
                        }
                    } else {
//                        continue;
                    }
                }

//                subImage = mainCam(ROI);
//                rm::ExtractColor(subImage, binary, request.OwnCamp, 40, true, {5, 5});
//                cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
//                rm::FindLightBlobs(contours, lightBlobs, 2, 10, 25, 80, 1500, subImage, true);
//                rm::FindArmour(lightBlobs, armours, 8, 24, 0.15, 0.45, 0.65, request.OwnCamp,
//                               cv::Size2f{(float) subImage.cols, (float) subImage.rows});
//                if (armours.empty()) {
//                    ROI = cv::Rect(0, 0, 0, 0);
//                } else {
//                    for (auto &armour: armours) {
//                        rm::SolvePNP(armour.vertices, cameraMatrix, distCoeffs, {5.5, 5.5}, tvecs, rvecs, ROI);
//                        if (rm::SolveDistance(tvecs) < 900) {
//                            double h = rm::SolveDeltaHeight(tvecs, request.GimbalPitch, {0, 0}, 0);
//                            rm::SolveShootFactor(tvecs, result, -9.8, request.FireRate, h - 0, {0, 0},
//                                                 rm::COMPENSATE_CLASSIC);
//
//                            std::cout << "pitch: " << result.pitchAngle << "  yaw: " << result.yawAngle << "  distance:"
//                                      << rm::SolveDistance(tvecs) << std::endl;
//                            ROI = rm::GetROI(armour.icon, 4, 1.5, {mainCam.cols, mainCam.rows}, ROI);
//                            message.push({result.pitchAngle, result.yawAngle, 0});
//
//                            rm::debug::DrawLightBlobs(lightBlobs, subImage, -1);
//                            rm::debug::DrawArmours(armours, subImage, -1);
//                            break;
//                        }
//                    }
//                }

                cv::imshow("frame", mainCam);
//                cv::imshow("binary", binary);
                cv::waitKey(1);
            }
        }
    });

    readThread.join();
    sendThread.join();
    detectionThread.join();
    return 0;
}
