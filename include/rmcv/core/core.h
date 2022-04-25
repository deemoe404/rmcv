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
    enum CampType {
        CAMP_RED = 0, CAMP_BLUE = 1, CAMP_NEUTRAL = -1
    };

    enum AimMode {
        AIM_COMBAT = 0, AIM_BUFF = 1
    };

    struct ShootFactor{
        float pitchAngle;
        float yawAngle;
        double estimateAirTime;
    };

    class LightBar {
    public:
        float angle = 0;                      // Rotation angle of the light bar (vertical when the angle is 90)
        rm::CampType camp = rm::CAMP_NEUTRAL; // Camp this light bar belongs to
        cv::Point2f center;                   // Mass center of the light bar
        cv::Point2f vertices[4];              // Four vertices around the light bar
        cv::Size2f size;                      // Width and height of the light bar

        LightBar() = default;

        explicit LightBar(cv::RotatedRect box, rm::CampType camp = rm::CAMP_NEUTRAL);
    };

    class Armour {
    public:
        float rank = 0;                          // A value help in sorting multiple armours
        rm::CampType camp = rm::CAMP_NEUTRAL;    // Camp this armour belongs to
        cv::Point2f icon[4];                     // Vertices of icon area
        cv::Point2f vertices[4];                 // Vertices of armour (square with light bar as side length for better PNP result)

        Armour() = default;

        explicit Armour(std::vector<rm::LightBar> lightBars, float rank = 0, rm::CampType camp = rm::CAMP_NEUTRAL);
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
