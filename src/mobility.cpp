//
// Created by yaione on 7/11/2022.
//

#include "mobility.h"

namespace rm
{
    void AxisRotateX(double y, double z, double thetaX, double& outY, double& outZ)
    {
        double y1 = y;
        double z1 = z;
        double rx = thetaX * CV_PI / 180;
        outY = cos(rx) * y1 - sin(rx) * z1;
        outZ = cos(rx) * z1 + sin(rx) * y1;
    }

    void AxisRotateY(double x, double z, double thetaY, double& outX, double& outZ)
    {
        double x1 = x;
        double z1 = z;
        double ry = thetaY * CV_PI / 180;
        outX = cos(ry) * x1 + sin(ry) * z1;
        outZ = cos(ry) * z1 - sin(ry) * x1;
    }

    void AxisRotateZ(double x, double y, double thetaZ, double& outX, double& outY)
    {
        double x1 = x;
        double y1 = y;
        double rz = thetaZ * CV_PI / 180;
        outX = cos(rz) * x1 - sin(rz) * y1;
        outY = sin(rz) * x1 + cos(rz) * y1;
    }

    double DeltaHeight(cv::InputArray translationVector, const double motorAngle, const cv::Point2f& offset,
                       const double angleOffset)
    {
        if (translationVector.kind() == cv::_InputArray::MAT)
        {
            cv::Mat tvecs = translationVector.getMat();

            double h = tvecs.ptr<double>(0)[1] - offset.y;
            double d = tvecs.ptr<double>(0)[2];

            double dPitch = -atan2(h, d) + (motorAngle - angleOffset);

            return d * tan(dPitch);
        }
        return NAN;
    }

    [[maybe_unused]] double Distance(cv::InputArray translationVector)
    {
        if (translationVector.kind() == cv::_InputArray::MAT)
        {
            cv::Mat tvecs = translationVector.getMat();
            return sqrt(pow(tvecs.at<double>(0), 2) + pow(tvecs.at<double>(1), 2) + pow(tvecs.at<double>(2), 2));
        }
        return NAN;
    }

    double ProjectileAngle(const double v0, const double g, const double d, const double h)
    {
        double a = (g * pow(d, 2.0)) / (2.0 * pow(v0, 2.0));
        double b = d;
        double c = a - h;

        double delta = pow(b, 2.0) - (4 * a * c);
        if (delta > 0)
        {
            double x1 = atan(((-1 * b) + sqrt(delta)) / (2 * a));
            double x2 = atan(((-1 * b) - sqrt(delta)) / (2 * a));
            return abs(x1) < abs(x2) ? x1 : x2;
        }
        else if (delta == 0)
        {
            return atan((-1) * (b / 2 * a));
        }

        return NAN;
    }

    [[maybe_unused]] bool
    SolveCameraPose(cv::InputArray rotationVector, cv::InputArray translationVector, cv::OutputArray pose)
    {
        if (translationVector.kind() == cv::_InputArray::MAT && rotationVector.kind() == cv::_InputArray::MAT)
        {
            pose.create({3, 1}, cv::_OutputArray::MAT);
            cv::Mat output = pose.getMat(), rvecs = rotationVector.getMat(), tvecs = translationVector.getMat();

            cv::Mat rotT = cv::Mat::eye(3, 3, CV_64F);
            cv::Rodrigues(tvecs, rotT);

            double rm[9];
            cv::Mat rotM(3, 3, CV_64FC1, rm);
            cv::Rodrigues(rvecs, rotM);
            double r11 = rotM.ptr<double>(0)[0];
            double r21 = rotM.ptr<double>(1)[0];
            double r31 = rotM.ptr<double>(2)[0];
            double r32 = rotM.ptr<double>(2)[1];
            double r33 = rotM.ptr<double>(2)[2];

            double thetaZ = atan2(r21, r11) / CV_PI * 180;
            double thetaY = atan2(-1 * r31, sqrt(r32 * r32 + r33 * r33)) / CV_PI * 180;
            double thetaX = atan2(r32, r33) / CV_PI * 180;

            double tx = tvecs.ptr<double>(0)[0];
            double ty = tvecs.ptr<double>(0)[1];
            double tz = tvecs.ptr<double>(0)[2];

            double x = tx, y = ty, z = tz;

            AxisRotateZ(x, y, -1 * thetaZ, x, y);
            AxisRotateY(x, z, -1 * thetaY, x, z);
            AxisRotateX(y, z, -1 * thetaX, y, z);

            output.ptr<float>(0)[0] = (float)thetaX * -1;
            output.ptr<float>(0)[1] = (float)thetaY * -1;
            output.ptr<float>(0)[2] = (float)thetaZ * -1;

            return true;
        }
        return false;
    }

    [[maybe_unused]] double
    SolveGEA(cv::InputArray translationVector, cv::OutputArray gimbalErrorAngle, const double g, const double v0,
             const double h, const cv::Point2f& offset, const double angleOffset, const rm::CompensateMode mode)
    {
        if (translationVector.kind() == cv::_InputArray::MAT)
        {
            cv::Mat tvecs = translationVector.getMat();
            double p = 0, t = 0, d = tvecs.ptr<double>(0)[2] / 100.0, y =
                       atan2(tvecs.ptr<double>(0)[0] - offset.x, tvecs.ptr<double>(0)[2]) * 180.0 / CV_PI;

            switch (mode)
            {
            case rm::COMPENSATE_NONE:
                p = -(atan2(tvecs.ptr<double>(0)[1] - offset.y, tvecs.ptr<double>(0)[2]) * 180.0 / CV_PI);
                t = d / v0;
                break;
            case COMPENSATE_CLASSIC:
                double normalAngle, centerAngle, targetAngle;
                normalAngle = atan2(h / 100.0, d) * 180.0 / CV_PI;
                centerAngle = -atan2(tvecs.ptr<double>(0)[1] - offset.y, tvecs.ptr<double>(0)[2]) * 180.0 / CV_PI;
                targetAngle = rm::ProjectileAngle(v0, g, d, h / 100.0) * 180.0 / CV_PI;

                p = (centerAngle - normalAngle + angleOffset * 180.0 / CV_PI) + targetAngle;
                t = d / abs(v0 * cos(targetAngle));
                break;
            case COMPENSATE_NI:
                return NAN; //TODO: Fix the bug of NI first!!!!
            }

            gimbalErrorAngle.create({2, 1}, cv::_OutputArray::MAT);
            cv::Mat gea = gimbalErrorAngle.getMat();
            gea.ptr<double>(0)[0] = p;
            gea.ptr<double>(0)[1] = y;
            return t;
        }

        return NAN;
    }

    std::tuple<cv::Mat, cv::Mat>
    solve_PnP(const cv::Point2f points_image[4], cv::InputArray cameraMatrix, cv::InputArray distortionFactor,
              const cv::Size2f& exactSize, const cv::Rect& ROI)
    {
        cv::Mat rotation_vector, translation_vector;

        const cv::Point2f offset(static_cast<float>(ROI.x), static_cast<float>(ROI.y));

        const std::vector exactPoint{
            cv::Point3f(-exactSize.width / 2.0f, exactSize.height / 2.0f, 0),
            cv::Point3f(exactSize.width / 2.0f, exactSize.height / 2.0f, 0),
            cv::Point3f(exactSize.width / 2.0f, -exactSize.height / 2.0f, 0),
            cv::Point3f(-exactSize.width / 2.0f, -exactSize.height / 2.0f, 0)
        };

        const std::vector coordinate{
            points_image[1] + offset, points_image[2] + offset,
            points_image[3] + offset, points_image[0] + offset
        };

        solvePnP(exactPoint, coordinate, cameraMatrix, distortionFactor, rotation_vector,
                 translation_vector, false, cv::SOLVEPNP_IPPE_SQUARE);

        return {rotation_vector, translation_vector};
    }
}
