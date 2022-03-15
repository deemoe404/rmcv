//
// Created by yaione on 3/5/22.
//

#include "rmcv/objdetect.h"

namespace rm {

    void FindLightBars(std::vector<std::vector<cv::Point>> &input, std::vector<rm::LightBar> &output, float minRatio,
                       float maxRatio, float tilAngle, float minArea) {
        if (input.size() < 2) return;
        output.clear();

        for (auto &i: input) {
            if (i.size() < 6 || cv::contourArea(i) < minArea) continue;

            cv::RotatedRect ellipse = cv::fitEllipseDirect(i);
            cv::RotatedRect box = cv::minAreaRect(i);

            // Aspect ratio
            float ratio = std::max(box.size.width, box.size.height) / std::min(box.size.width, box.size.height);
            if (ratio > maxRatio || ratio < minRatio) continue;

            // Angle (Perpendicular to the frame is considered as 90 degrees, left < 90, right > 90)
            float angle = ellipse.angle > 90 ? ellipse.angle - 90 : ellipse.angle + 90;
            if (abs(angle - 90) > tilAngle) continue;

            output.emplace_back(box, angle);
        }
    }

    void
    FindArmour(std::vector<rm::LightBar> &input, std::vector<rm::Armour> &output, float maxAngleDif, float errAngle,
               float minBoxRatio, float maxBoxRatio, float lenRatio, cv::Size frameSize) {
        if (input.size() < 2) return;
        output.clear();

        for (int i = 0; i < input.size() - 1; i++) {
            for (int j = i + 1; j < input.size(); j++) {
                // Poor angle between two light bar
                float angleDif = abs(input[i].angle - input[j].angle);
                if (angleDif > maxAngleDif) continue;

                int y = abs(input[i].center.y - input[j].center.y);
                int x = abs(input[i].center.x - input[j].center.x);
                float angle = (float) atan2(y, x) * 180 / (float) CV_PI;
                float errorI = abs(
                        input[i].angle > 90 ? abs(input[i].angle - angle) - 90 : abs(180 - input[i].angle - angle) -
                                                                                 90);
                float errorJ = abs(
                        input[j].angle > 90 ? abs(input[j].angle - angle) - 90 : abs(180 - input[j].angle - angle) -
                                                                                 90);
                if (errorI > errAngle || errorJ > errAngle) continue;

                float heightI = input[i].size.height;
                float heightJ = input[j].size.height;
                float ratio = std::min(heightI, heightJ) / std::max(heightI, heightJ);
                if (ratio < lenRatio) continue;

                float distance = rm::PointDistance(input[i].center, input[j].center);
                float boxRatio = ((heightI + heightJ) / 2) / distance;
                if (boxRatio > maxBoxRatio || boxRatio < minBoxRatio)continue;

                cv::Point centerArmour((input[i].center.x + input[j].center.x) / 2,
                                       (input[i].center.y + input[j].center.y) / 2);
                cv::Point centerFrame(frameSize.width / 2, frameSize.height / 2);

                output.push_back(rm::Armour({input[i], input[j]}, rm::ARMOUR_SMALL,
                                            rm::PointDistance(centerArmour, centerFrame)));
            }
        }

        if (!output.empty()) {
            std::sort(output.begin(), output.end(), [](rm::Armour armour1, rm::Armour armour2) {
                return armour1.distance2D < armour2.distance2D;
            });
        }
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

    void SolveArmourPose(rm::Armour &target, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Size2f exactSize) {
        std::vector<cv::Point3f> exactPoint{cv::Point3f(-exactSize.width / 2.0f, exactSize.height / 2.0f, 0),
                                            cv::Point3f(exactSize.width / 2.0f, exactSize.height / 2.0f, 0),
                                            cv::Point3f(exactSize.width / 2.0f, -exactSize.height / 2.0f, 0),
                                            cv::Point3f(-exactSize.width / 2.0f, -exactSize.height / 2.0f, 0)};

        std::vector<cv::Point2f> tdCoordinate{target.vertices[1], target.vertices[2], target.vertices[3],
                                              target.vertices[0]};

        cv::solvePnP(exactPoint, tdCoordinate, cameraMatrix, distCoeffs, target.rvecs, target.tvecs, false,
                     cv::SOLVEPNP_IPPE_SQUARE);
    }
}
