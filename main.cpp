#include <iostream>
#include <thread>
#include "rmcv/rmcv.h"

int main() {
    DahengCamera camera;
    cv::Mat frames;
    int key;

    if (camera.dahengCameraInit((char *) "KE0210030295", 2000, 210)) {
        while (key != 'q') {
            frames = camera.getFrame();

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
            rm::FindArmour(lbs, ams, 25, 18, 0.325, 0.6, 0.75);
            std::vector<std::vector<cv::Point>> tmp3;
            for (auto &am: ams) {
                std::vector<cv::Point> tmp2;
                tmp2.push_back(am.vertices[0]);
                tmp2.push_back(am.vertices[1]);
                tmp2.push_back(am.vertices[2]);
                tmp2.push_back(am.vertices[3]);
                tmp3.push_back(tmp2);
                cv::drawContours(frames, tmp3, -1, {0, 255, 255}, 1);
            }

            for (auto &am: ams) {
//                cv::Mat m2 = frames(am.box);
//
                cv::Mat calcal;
//                cv::Point2f test[3];
//                test[0] = cv::Point2f(0, 0);
//                test[1] = cv::Point2f(0, 15);
//                test[2] = cv::Point2f(15, 15);
////                test[3] = cv::Point2f(15, 0);
                cv::Point2f pts1[4];
                cv::Point2f pts2[3];

                pts1[0] = am.vertices[0];
                pts1[1] = am.vertices[3];
                pts1[2] = am.vertices[1];
//                pts1[3] = cv::Point2f( 0, 225 );

                pts2[0] = cv::Point2f( 0, 0);
                pts2[1] = cv::Point2f( 225, 0 );
                pts2[2] = cv::Point2f( 0, 225 );

                rm::CalcRatio(frames, calcal, pts1, pts2);
                rm::CalcGamma(calcal, calcal, 0.25);
                cv::imshow("aa", calcal);
            }


            cv::imshow("bin", bin);
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
