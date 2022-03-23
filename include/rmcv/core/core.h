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
        FORCE_HERO = 0,       // Big armour 1
        FORCE_ENGINEER = 1,   // Small armour 2
        FORCE_STANDARD_3 = 2, // Small armour 3
        FORCE_STANDARD_4 = 3, // Small armour 4
        FORCE_STANDARD_5 = 4, // Small armour 5
        FORCE_SENTRY = 5,     // Big armour icon
        BUILD_BASE = 6,       // Big armour icon
        BUILD_OUTPOST = 7,    // Small armour icon
        FORCE_UNKNOWN = 8
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
        cv::Point vertices[4];    // Vertices around two light bars
        cv::Point icon[4];        // Vertices around icon
        cv::Rect iconBox;         // Icon rect
        double distance2D = 0;    // Distance to the center of the frame
        double airTime = 0;       // Estimate time before hitting target
        float pitch = 0, yaw = 0; // Angle error in pitch & yaw
        char rank = -1;           // Help full-auto force in making decision
        rm::ForceType forceType = rm::FORCE_STANDARD_5;
        rm::ArmourType armourType = rm::ARMOUR_SMALL;
        rm::CampType campType = rm::CAMP_BLUE;
        cv::Mat rvecs;
        cv::Mat tvecs;
        cv::Point2f error;


        explicit Armour(std::vector<rm::LightBar> lightBars, rm::ArmourType armourType = rm::ARMOUR_SMALL,
                        rm::CampType campType = rm::CAMP_BLUE, double distance2D = 0);
    };

    class Package {
    public:
        rm::CampType camp = rm::CAMP_RED;  // Self camp
        rm::AimMode mode = rm::AIM_COMBAT; // Aim mode
        unsigned char speed = 0;           // Bullet speed
        float pitch = 0;                   // Pitch angle
        cv::Mat frame;
        cv::Mat binary;
        std::vector<rm::Armour> armours;

        Package(rm::CampType camp, rm::AimMode mode, unsigned char speed, float pitch, const cv::Mat &inputFrame,
                const cv::Mat &inputBinary);

        explicit Package(const shared_ptr<rm::Package> &input);
    };

}

#endif //RM_STANDARD2022_CORE_H
