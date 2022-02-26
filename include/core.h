//
// Created by yaione on 2/26/2022.
//

#ifndef RM_STANDARD2022_CORE_H
#define RM_STANDARD2022_CORE_H

#include "opencv2/opencv.hpp"

namespace rm {
    enum ForceType {
        FORCE_HERO = 0,
        FORCE_STANDARD = 1,
        FORCE_ENGINEER = 2,
        FORCE_AERIAL = 3,
        FORCE_SENTRY = 4
    };

    enum CampType {
        CAMP_RED = 0,
        CAMP_BLUE = 1
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
        float rank = -1;

        Armour(std::vector<rm::LightBar> lightBars);
    };

}

#endif //RM_STANDARD2022_CORE_H
