//
// Created by yaione on 3/5/22.
//

#include "rmcv/objdetect.h"

namespace rm {

    void FindLightBars(std::vector<std::vector<cv::Point>> &input, std::vector<rm::LightBar> &output, float minRatio,
                       float maxRatio, float minAngle, float maxAngle, float minArea) {

    }

    void FindArmour(std::vector<rm::LightBar> &input, std::vector<rm::Armour> &output, float angleDif, float angleErr,
                    float minBoxRatio, float maxBoxRatio, float duoRatio) {

    }

    void SolveAirTrack(rm::Armour &input, double g, double v0, double hOffset, float motorAngle) {
        double h = (input.tvecs.ptr<double>(0)[1] - hOffset) / 100;
        double d = input.tvecs.ptr<double>(0)[2] / 100;

        double dPitch = -atan2(h, d) + motorAngle;               // Horizontal related angle.
        double hWorld = d * tan(dPitch);                            // Ground related height.
        double shootTheta = rm::NewtonIteration(rm::ProjectileMotionFD, {g, d, hWorld, v0});

        input.pitch = (float) shootTheta - motorAngle;
        input.yaw = (float) atan2(input.tvecs.ptr<double>(0)[0], input.tvecs.ptr<double>(0)[2]);
        input.airTime = d / v0 * cos(shootTheta);
    }

    void SolveArmourPose(Armour &target, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Point2f &exactSize) {
        target.rvecs = cv::Mat::zeros(3, 1, CV_64FC1);
        target.tvecs = cv::Mat::zeros(3, 1, CV_64FC1);

        std::vector<cv::Point3f> exactPoint{cv::Point3f(0, 0, 0), cv::Point3f(0, 0, exactSize.y),
                                            cv::Point3f(exactSize.x, 0, exactSize.y), cv::Point3f(exactSize.x, 0, 0)};

        std::vector<cv::Point2f> tdCoordinate{cv::Point2f(target.vertices[0].x, target.vertices[0].y),
                                              cv::Point2f(target.vertices[1].x, target.vertices[1].y),
                                              cv::Point2f(target.vertices[2].x, target.vertices[2].y),
                                              cv::Point2f(target.vertices[3].x, target.vertices[3].y)};

        cv::solvePnP(exactPoint, tdCoordinate, cameraMatrix, distCoeffs, target.rvecs, target.tvecs, false,
                     cv::SOLVEPNP_SQPNP);
    }
}
