//
// Created by yaione on 3/1/2022.
//

#ifndef RM_STANDARD2022_OBJDETECT_H
#define RM_STANDARD2022_OBJDETECT_H

#include "core/core.h"

namespace rm {
    void FindLightBars(std::vector<std::vector<cv::Point>> &input, std::vector<rm::LightBar> &output, float minRatio,
                       float maxRatio, float tilAngle, float minArea, rm::CampType camp = rm::CAMP_RED);

    /// Fit light bars from a set of contours. Auto detect light bar color.
    /// \param contours The set of contours.
    /// \param output Output light bars.
    /// \param minRatio Minimal aspect ratio.
    /// \param maxRatio Maximal aspect ratio.
    /// \param tiltAngle Maximal tilt angle.
    /// \param minArea Minimal contour area.
    /// \param frame Source frame.
    /// \param useFitEllipse Use cv::fitEllipseDirect to define light bar outlines instead of cv::minAreaRect when set
    ///                      to true. Usually recommended when source frame was over exposed.
    void
    FindLightBars(std::vector<std::vector<cv::Point>> &contours, std::vector<rm::LightBar> &lightBars, float minRatio,
                  float maxRatio, float tiltAngle, float minArea, float maxArea, cv::Mat &frame,
                  bool useFitEllipse = true);

    void
    FindArmour(std::vector<rm::LightBar> &input, std::vector<rm::Armour> &output, float maxAngleDif, float errAngle,
               float minBoxRatio, float maxBoxRatio, float lenRatio, rm::CampType ownCamp,
               cv::Size frameSize = {1920, 1080});

    /// Use PNP algorithm to estimate the pose in 3D of Armour.
    /// \param target Armour which changes is to make on.
    /// \param cameraMatrix
    /// \param distCoeffs
    /// \param exactSize Real size of the Armour, unit in mm.
    void SolveArmourPose(rm::Armour &target, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Size2f exactSize);

    /// Calculate the delta pitch, delta yaw and estimated time on air using Newton Iteration.
    /// \param target Armour which changes is to make on.
    /// \param g Acceleration of gravity.
    /// \param v0 The initial speed of bullet.
    /// \param hOffset Distance between camera and barrel, positive when barrel sets under camera.
    /// \param motorAngle Positive upwards, unit in radians.
    void SolveAirTrack(rm::Armour &target, double g, double v0, double hOffset, float motorAngle);

    /// Calculate the delta pitch, delta yaw and estimated time on air with no compensate.
    /// \param target Armour which changes is to make on.
    /// \param v0 The initial speed of bullet.
    /// \param hOffset Distance between camera and barrel, positive when barrel sets under camera.
    /// \param wOffset
    void SolveAirTrack(rm::Armour &target, double v0, double hOffset = 0, double wOffset = 0);

    void SolveCameraPose(cv::Mat &rvecs, cv::Mat &tvecs, cv::Mat &output);
}

#endif //RM_STANDARD2022_OBJDETECT_H
