//
// Created by yaione on 2/26/2022.
//

#ifndef RM_STANDARD2022_CORE_H
#define RM_STANDARD2022_CORE_H

#include "opencv2/opencv.hpp"
#include "daheng/daheng.h"
#include "utils.h"
#include "serialport.h"
#include "parallequeue.hpp"

namespace rm {
    enum ArmourType {
        ARMOUR_BIG = 0, ARMOUR_SMALL = 1
    };

    enum ForceType {
        FORCE_HERO = 0, FORCE_STANDARD = 1, FORCE_ENGINEER = 2, FORCE_AERIAL = 3, FORCE_SENTRY = 4
    };

    enum CampType {
        CAMP_RED = 0, CAMP_BLUE = 1
    };

    enum AimMode {
        AIM_COMBAT = 0, AIM_BUFF = 1
    };

    class LightBar {
    public:
        cv::Point vertices[4]; // Four vertices around light bar
        cv::Point center;
        cv::Size2f size;
        float angle;

        LightBar(cv::RotatedRect box, float angle);
    };

    class Armour {
    public:
        cv::Point vertices[4]; // Vertices around two light bars
        cv::Point icon[4];     // Vertices around icon
        cv::Rect box;          // Bounding rect
        cv::Rect iconBox;      // Icon rect
        cv::Mat rvecs;
        cv::Mat tvecs;
        rm::ForceType forceType = rm::FORCE_STANDARD;
        rm::ArmourType armourType = rm::ARMOUR_SMALL;
        double airTime = 0;
        double distance2D = 0;
        float pitch = 0;
        float yaw = 0;
        char rank = -1;

        Armour(std::vector<rm::LightBar> lightBars, rm::ArmourType armourType = rm::ARMOUR_SMALL,
                        double distance2D = 0);
    };

    class Package {
    public:
        rm::CampType camp = rm::CAMP_RED;  // Self camp
        rm::AimMode mode = rm::AIM_COMBAT; // Aim mode
        unsigned char speed = 0;                    // Bullet speed
        float pitch = 0;                   // Pitch angle
        cv::Mat frame;
        cv::Mat binary;
        std::vector<rm::Armour> armours;

        Package(rm::CampType camp, rm::AimMode mode, unsigned char speed, float pitch, const cv::Mat &inputFrame,
                const cv::Mat &inputBinary);
    };

}

#endif //RM_STANDARD2022_CORE_H
