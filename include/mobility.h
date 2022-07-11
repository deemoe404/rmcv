///
/// \file      mobility.h
/// \brief     Provides a series of functions for solving automatic control parameters.
/// \version   v0.1.0
/// \date      7/11/2022
/// \author    EarthOL-088217
/// \copyright Copyright (c) 2020-2022 and all right reserved.
///

#ifndef RMCV_MOBILITY_H
#define RMCV_MOBILITY_H

#include "core/core.h"

namespace rm {
    /// \brief Define witch method is used to calculate the compensation of gravity.
    [[maybe_unused]] typedef enum CompensateMode {
        COMPENSATE_NONE = 0,    ///< No compensation
        COMPENSATE_CLASSIC = 1, ///< Use newtonian's theorem of mechanics
        COMPENSATE_NI = 2       ///< Use newton iteration method
    } CompensateMode;

    /// \brief Rotate the vector around the x-axis.
    /// \param y      y of the vector.
    /// \param z      z of the vector.
    /// \param thetaX angle to be rotate.
    /// \param outY   y of the vector after rotation.
    /// \param outZ   z of the vector after rotation.
    void AxisRotateX(double y, double z, double thetaX, double &outY, double &outZ);

    /// \brief Rotate the vector around the y-axis.
    /// \param x      x of the vector.
    /// \param z      z of the vector.
    /// \param thetaY angle to be rotate.
    /// \param outX   x of the vector after rotation.
    /// \param outZ   z of the vector after rotation.
    void AxisRotateY(double x, double z, double thetaY, double &outX, double &outZ);

    /// \brief Rotate the vector around the z-axis.
    /// \param x      x of the vector.
    /// \param y      y of the vector.
    /// \param thetaZ angle to be rotate.
    /// \param outX   x of the vector after rotation.
    /// \param outY   y of the vector after rotation.
    void AxisRotateZ(double x, double y, double thetaZ, double &outX, double &outY);

    /// \brief Solve height difference between barrel and target.
    /// \param translationVector The translation vector of target.
    /// \param motorAngle        The motor angle of gimbal, positive upwards. (RAD)
    /// \param offset            Offset between camera and barrel.            (cm)
    /// \param angleOffset       Angle offset between camera and barrel.      (RAD)
    /// \return Height difference between barrel and target in cm, NAN if translationVector is not in cv::Mat format.
    [[maybe_unused]] double
    DeltaHeight(cv::InputArray translationVector, double motorAngle, const cv::Point2f &offset = {0, 0},
                double angleOffset = 0);

    /// \brief Solve the distance between camera and target.
    /// \param translationVector The translation vector of target.
    /// \return Distance in cm, NAN if translationVector is not in cv::Mat format.
    [[maybe_unused]] double Distance(cv::InputArray translationVector);

    /// \brief Solve initial angle required for a projectile motion.
    ///
    /// This function would be done by solving \f$ 0 = \frac{g \cdot d^{2}}{2 \cdot v_{0}^{2}} \cdot tan\theta^{2} + d
    /// \cdot tan\theta + (\frac{g \cdot d^{2}}{2 \cdot v_{0}^{2}} - h) \f$ for \f$tan\theta\f$ using the binary first-order
    /// root formula, witch can be deduced from Newtonian's theorem of mechanics.
    ///
    /// \param v0 Initial launch speed.    (m/s)
    /// \param g  Acceleration of gravity. (m/s^2)
    /// \param d  Horizontal distance.     (m)
    /// \param h  Height difference.       (m)
    /// \return Estimated launch angle in radians, NAN if equations have no real solutions.
    double ProjectileAngle(double v0, double g, double d, double h);

    /// \brief Solve camera pose in pitch, yaw, and roll relative to a target.
    /// \param rotationVector    The rotation vector of target.
    /// \param translationVector The translation vector of target.
    /// \param pose              [OUT] Camera pose.
    /// \return False if translationVector or rotationVector is not in cv::Mat format.
    [[maybe_unused]] bool
    SolveCameraPose(cv::InputArray rotationVector, cv::InputArray translationVector, cv::OutputArray pose);

    /// \brief Solve gimbal error angle to target by given method.
    /// \param translationVector Translation vector of target.
    /// \param gimbalErrorAngle  [OUT] Estimation error angle. Format: [pitch, yaw].
    /// \param g                 Acceleration of gravity.                (m/s^2)
    /// \param v0                The initial speed of bullet.            (m/s)
    /// \param h                 Height between barrel and target.       (m)
    /// \param offset            Offset between camera and barrel.       (cm)
    /// \param angleOffset       Angle offset between camera and barrel. (RAD)
    /// \param mode              Method to be used to calculate the compensation of gravity.
    /// \return Estimation air time, NAN if translationVector is not in cv::Mat format.
    [[maybe_unused]] double
    SolveGEA(cv::InputArray translationVector, cv::OutputArray gimbalErrorAngle, double g, double v0, double h,
             const cv::Point2f &offset = {0, 0}, double angleOffset = 0, rm::CompensateMode mode = rm::COMPENSATE_NONE);

    /// \brief Solve the rotation & translation vector using cv::solvePnP & cv::SOLVEPNP_IPPE_SQUARE.
    /// \param imagePoints       Object points on image (Quantity must be four).
    /// \param cameraMatrix      Camera matrix.
    /// \param distortionFactor  Camera distortion factor.
    /// \param exactSize         Exact size of the coordinate object (cm).
    /// \param translationVector [OUT] Translation vector.
    /// \param rotationVector    [OUT] Rotation vector.
    /// \return False if imagePoints.size() is not 4, or there might be errors in Camera-parameters or Points-given, such
    ///         as wrong There is no one-to-one correspondence between the points on the image and the actual points.
    [[maybe_unused]] bool
    SolvePNP(const std::vector<cv::Point2f> &imagePoints, cv::InputArray cameraMatrix, cv::InputArray distortionFactor,
             const cv::Size2f &exactSize, cv::OutputArray translationVector, cv::OutputArray rotationVector,
             const cv::Rect &ROI = {0, 0, 0, 0});
}

#endif //RMCV_MOBILITY_H
