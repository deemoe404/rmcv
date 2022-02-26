//
// Created by yaione on 2/26/2022.
//

#ifndef RM_STANDARD2022_CORE_H
#define RM_STANDARD2022_CORE_H

#include "opencv2/opencv.hpp"

namespace rm {
    enum ForceType {
        HERO = 0,
        STANDARD = 1,
        ENGINEER = 2,
        AERIAL = 3,
        SENTRY = 4
    };

    class LightBar {
    public:
        cv::Point2f vertices[4];
        cv::Point2f center;
        cv::Size2f size;
        float angle;

        LightBar(cv::RotatedRect ellipse);
    };

    class Armour {
    public:
        cv::Point2f vertices[4];
        cv::Point2f center;
        cv::Size2f size;
        float rank;

        Armour(std::vector<rm::LightBar> lightBars);
    };

}

#endif //RM_STANDARD2022_CORE_H
