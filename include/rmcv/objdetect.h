//
// Created by yaione on 3/1/2022.
//

#ifndef RM_STANDARD2022_OBJDETECT_H
#define RM_STANDARD2022_OBJDETECT_H

#include "core/core.h"

namespace rm {
    void FindLightBars(std::vector<std::vector<cv::Point>> &input, std::vector<rm::LightBar> &output, float minRatio,
                       float maxRatio, float tilAngle, float minArea);

    void
    FindArmour(std::vector<rm::LightBar> &input, std::vector<rm::Armour> &output, float maxAngleDif, float errAngle,
               float minBoxRatio, float maxBoxRatio, float lenRatio, cv::Size frameSize = {1920, 1080},
               rm::CampType campType = rm::CAMP_BLUE);

    /// Use PNP algorithm to estimate the pose in 3D of Armour.
    /// \param target Armour which changes is to make on.
    /// \param cameraMatrix
    /// \param distCoeffs
    /// \param exactSize Real size of the Armour, unit in mm.
    void SolveArmourPose(rm::Armour &target, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Size2f exactSize);
    // TODO: to size

    /// Calculate the delta pitch, delta yaw and estimated time on air.
    /// \param target Armour which changes is to make on.
    /// \param g Acceleration of gravity.
    /// \param v0 The initial speed of bullet.
    /// \param hOffset Distance between camera and barrel, positive when barrel sets under camera.
    /// \param motorAngle Positive upwards, unit in radians.
    void SolveAirTrack(rm::Armour &target, double g, double v0, double hOffset, float motorAngle);
}

#endif //RM_STANDARD2022_OBJDETECT_H
