//
// Created by yaione on 3/1/2022.
//

#ifndef RM_STANDARD2022_OBJDETECT_H
#define RM_STANDARD2022_OBJDETECT_H

#include "core/core.h"

namespace rm {
    void FindLightBars(std::vector<std::vector<cv::Point>> &input, std::vector<rm::LightBar> &output, float minRatio,
                       float maxRatio, float minAngle, float maxAngle, float minArea);

    void FindArmour(std::vector<rm::LightBar> &input, std::vector<rm::Armour> &output, float angleDif, float angleErr,
                    float minBoxRatio, float maxBoxRatio, float duoRatio);

    void SolveArmourPose(rm::Armour &target, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);

    double SolveAirTrack(rm::Armour &input, double g, double v0, double hOffset);
}

#endif //RM_STANDARD2022_OBJDETECT_H
