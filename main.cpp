#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

int main() {
    DahengCamera camera;
    int key;

    if (camera.dahengCameraInit((char *) "KE0210030295", 2500, 210)) {
        while (key != 'q') {
            cv::Mat frames = camera.getFrame();
            std::vector<cv::Mat> channels;
            cv::split(frames, channels);
            cv::Mat gray = channels[0] - channels[2];
            cv::Mat bin;

            // Blue: bin[140,255], area[10), lb{ratio[3,15], ta(20]}
            cv::inRange(gray, 140, 255, bin);
            auto kernal = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
            cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, kernal);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE); // no roi

//            for(int i = 0;i < contours.size();i++){
//                if(cv::contourArea(contours[i]) < 10) continue;
//                printf("%f\n", cv::contourArea(contours[i]));
//                cv::drawContours(frames, contours, i, cv::Scalar(0, 0, 255), 1);
//            }

            std::vector<rm::LightBar> lbs;
            rm::FindLightBars(contours, lbs, 3, 19, 20, 10);
            std::vector<std::vector<cv::Point>> tmp;
            for (auto &lb: lbs) {
                std::vector<cv::Point> tmp2;
                tmp2.push_back(lb.vertices[0]);
                tmp2.push_back(lb.vertices[1]);
                tmp2.push_back(lb.vertices[2]);
                tmp2.push_back(lb.vertices[3]);
                tmp.push_back(tmp2);
            }
            cv::drawContours(frames, tmp, -1, {0, 0, 255}, 1);

            std::vector<rm::Armour> ams;
            rm::FindArmour(lbs, ams, 20, 5, 0.25, 0.7, 0.8);
            std::vector<std::vector<cv::Point>> tmp3;
            for (auto &am: ams) {
                std::vector<cv::Point> tmp2;
                tmp2.push_back(am.vertices[0]);
                tmp2.push_back(am.vertices[1]);
                tmp2.push_back(am.vertices[2]);
                tmp2.push_back(am.vertices[3]);
                tmp3.push_back(tmp2);
                cv::drawContours(frames, tmp3, -1, {0, 255, 255}, 3);
            }

            for (auto &am: ams) {
                cv::Mat calcal;

                rm::CalcRatio(frames, calcal, am.vertices, am.box, {45, 45});
                rm::CalcGamma(calcal, calcal, 0.15);

                std::vector<cv::Mat> channels2;
                cv::split(calcal, channels2);
                cv::Mat grayAM;
                cv::cvtColor(calcal, grayAM, COLOR_BGR2GRAY);
                cv::Mat binAM;
                cv::inRange(grayAM, 50, 255, binAM);
                cv::imshow("aa", grayAM);
            }


//            cv::imshow("bin", bin);
            cv::imshow("frame", frames);

            key = cv::waitKey(1);
        }
    }

//    rm::SerialPort sp{};
//    sp.Initialize();
//
//    rm::Response response{};
//    response.yaw.data = 20.0;
//    response.pitch.data = 30.0;
//    response.rank = 3;
//
//    rm::Request request{};
//
//    thread t1([&]() {
//        while (true) {
//            std::cout << "send status: " << sp.Send(response) << std::endl;
//        }
//    });
//
//    thread t2([&]() {
//        while (true) {
//            if (sp.Receive(request)) {
//                std::cout << "received: ";
//                std::cout << "p: " << request.pitch.data << "  ";
//                std::cout << "camp: " << request.camp << "  ";
//                std::cout << "mode: " << request.mode << "  ";
//                std::cout << "speed: " << request.speed << std::endl;
//            }
//        }
//    });
//
//    t1.join();
//    t2.join();
//
//    return 0;
}
