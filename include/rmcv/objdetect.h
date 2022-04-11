//
// Created by yaione on 3/1/2022.
//

#ifndef RM_STANDARD2022_OBJDETECT_H
#define RM_STANDARD2022_OBJDETECT_H

#include "core/core.h"

namespace rm {
    enum CompensateMode {
        COMPENSATE_NONE = 0,    // No compensate
        COMPENSATE_CLASSIC = 1, // Use rm::NewtonIteration & rm::ProjectileMotionFD for compensate
        COMPENSATE_NI = 2       // Use classic Newton's laws of motion for compensate
    };

    /// Judge a contour if it's a light bar with the given qualifications.
    /// \param contour The contour.
    /// \param minRatio Minimal aspect ratio.
    /// \param maxRatio Maximal aspect ratio.
    /// \param tiltAngle Maximal tilt angle.
    /// \param minArea Minimal contour area.
    /// \param maxArea Maximal contour area.
    /// \param useFitEllipse Use cv::fitEllipseDirect to define light bar outlines instead of cv::minAreaRect when set
    ///                      to true. Usually recommended when source frame was over exposed.
    bool
    JudgeLightBar(const std::vector<cv::Point> &contour, float minRatio, float maxRatio, float tilAngle, float minArea,
                  float maxArea, bool useFitEllipse = true);

    /// Fit light bars from a set of contours with the given color. (Each contours must contains more than 6 points.)
    /// \param contours The set of contours.
    /// \param lightBars Output light bars.
    /// \param minRatio Minimal aspect ratio.
    /// \param maxRatio Maximal aspect ratio.
    /// \param tiltAngle Maximal tilt angle.
    /// \param minArea Minimal contour area.
    /// \param maxArea Maximal contour area.
    /// \param frame Source frame. Used for auto color detection.
    /// \param useFitEllipse Use cv::fitEllipseDirect to define light bar outlines instead of cv::minAreaRect when set
    ///                      to true. Usually recommended when source frame was over exposed.
    void
    FindLightBars(std::vector<std::vector<cv::Point>> &contours, std::vector<rm::LightBar> &lightBars, float minRatio,
                  float maxRatio, float tiltAngle, float minArea, float maxArea, rm::CampType camp = rm::CAMP_RED,
                  bool useFitEllipse = true);

    /// Fit light bars from a set of contours. Auto detect light bar color. (Each contours must contains more than 6
    /// points.)
    /// \param contours The set of contours.
    /// \param lightBars Output light bars.
    /// \param minRatio Minimal aspect ratio.
    /// \param maxRatio Maximal aspect ratio.
    /// \param tiltAngle Maximal tilt angle.
    /// \param minArea Minimal contour area.
    /// \param maxArea Maximal contour area.
    /// \param frame Source frame. Used for auto color detection.
    /// \param useFitEllipse Use cv::fitEllipseDirect to define light bar outlines instead of cv::minAreaRect when set
    ///                      to true. Usually recommended when source frame was over exposed.
    void
    FindLightBars(std::vector<std::vector<cv::Point>> &contours, std::vector<rm::LightBar> &lightBars, float minRatio,
                  float maxRatio, float tiltAngle, float minArea, float maxArea, cv::Mat &frame,
                  bool useFitEllipse = true);

    /// Fit light bars from a binary image. Auto detect light bar color. (Each contours must contains more than 6
    /// points.)
    /// \param binary Source binary image. This function would consider the external contours only.
    /// \param lightBars Output light bars.
    /// \param minRatio Minimal aspect ratio.
    /// \param maxRatio Maximal aspect ratio.
    /// \param tiltAngle Maximal tilt angle.
    /// \param minArea Minimal contour area.
    /// \param maxArea Maximal contour area.
    /// \param frame Source frame. Used for auto color detection.
    /// \param useFitEllipse Use cv::fitEllipseDirect to define light bar outlines instead of cv::minAreaRect when set
    ///                      to true. Usually recommended when source frame was over exposed.
    void FindLightBars(cv::Mat &binary, std::vector<rm::LightBar> &lightBars, float minRatio, float maxRatio,
                       float tiltAngle, float minArea, float maxArea, cv::Mat &frame, bool useFitEllipse = true);

    /// Detect if there is a interference over all light bars at the given pair of light bars.
    /// \param lightBars All light bars on the frame.
    /// \param leftIndex Index of the left light bar.
    /// \param rightIndex Index of the right light bar.
    /// \return True if there is a interference;
    bool LightBarInterference(std::vector<rm::LightBar> &lightBars, int leftIndex, int rightIndex);

    /// Fit armours from a set of light bars.
    /// \param lightBars The set of light bars
    /// \param armours Output armours
    /// \param maxAngleDif Maximum angle difference between two light bars.
    /// \param errAngle Maximum angle between the over all rect and the two light bars.
    /// \param minBoxRatio Minimum ratio of the armour box.
    /// \param maxBoxRatio Maximum ratio of the armour box.
    /// \param lenRatio Maximum length ratio between two light bars.
    /// \param ownCamp Own camp.
    /// \param frameSize Frame size. When this parameter is specified, the distance between armour and the frame center
    ///                  would be used to sort the output.
    void FindArmour(std::vector<rm::LightBar> &lightBars, std::vector<rm::Armour> &armours, float maxAngleDif,
                    float errAngle, float minBoxRatio, float maxBoxRatio, float lenRatio, rm::CampType ownCamp,
                    cv::Size2f frameSize);

    /// Fit armours from a set of light bars.
    /// \param lightBars The set of light bars
    /// \param armours Output armours
    /// \param maxAngleDif Maximum angle difference between two light bars.
    /// \param errAngle Maximum angle between the over all rect and the two light bars.
    /// \param minBoxRatio Minimum ratio of the armour box.
    /// \param maxBoxRatio Maximum ratio of the armour box.
    /// \param lenRatio Maximum length ratio between two light bars.
    /// \param ownCamp Own camp.
    /// \param attention Attention point. When this parameter is specified, the distance between armour and the
    ///                  attention point would be used to sort the output.
    void FindArmour(std::vector<rm::LightBar> &lightBars, std::vector<rm::Armour> &armours, float maxAngleDif,
                    float errAngle, float minBoxRatio, float maxBoxRatio, float lenRatio, rm::CampType ownCamp,
                    cv::Point2f attention);

    /// Use PNP algorithm to estimate the pose in 3D of Armour.
    /// \param target Armour which changes is to make on.
    /// \param cameraMatrix
    /// \param distCoeffs
    /// \param exactSize Real size of the Armour, unit in mm.
//    void SolveArmourPose(rm::Armour &target, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Size2f exactSize);

    /// Calculate the shoot factor using Newton Iteration.
    /// \param target The target armour.
    /// \param shootFactor Output shoot factor.
    /// \param g Acceleration of gravity.
    /// \param v0 The initial speed of bullet.
    /// \param hOffset Distance between camera and barrel, positive when barrel sets under camera.
    /// \param motorAngle Positive upwards, unit in radians.
//    void SolveShootFactor(rm::Armour &target, rm::ShootFactor &shootFactor, double g, double v0, double hOffset,
//                          float motorAngle);

    /// Calculate the shoot factor without compensate.
    /// \param target The target armour.
    /// \param shootFactor Output shoot factor.
    /// \param v0 The initial speed of bullet.
    /// \param hOffset Distance between camera and barrel & X error of PNP. (positive when barrel sets under camera)
    /// \param wOffset Y error of PNP.
//    void SolveShootFactor(rm::Armour &target, rm::ShootFactor &shootFactor, double v0, double hOffset = 0,
//                          double wOffset = 0);

    /// Calculate the shoot factor of given target.
    /// \param target The target armour.
    /// \param shootFactor Output shoot factor.
    /// \param v0 The initial speed of bullet.
    /// \param vOffset Vertical distance between camera and barrel & Vertical error of PNP. (positive when barrel sets
    ///                under camera)
    /// \param hOffset Horizontal error of PNP.
    /// \param g Acceleration of gravity.
    /// \param mode Compensate mode.
    /// \param pitchAngle Pitch angle of pitch-motor. Positive upwards. (radians)
    /// \param height MotorAngle is ignored if this parameter is specified. (centimeter)
    void SolveShootFactor(rm::Armour &target, rm::ShootFactor &shootFactor, double v0, double vOffset, double hOffset,
                          double g, rm::CompensateMode mode, float pitchAngle = 0, double height = NAN);

    void SolveCameraPose(cv::Mat &rvecs, cv::Mat &tvecs, cv::Mat &output);
}

#endif //RM_STANDARD2022_OBJDETECT_H
