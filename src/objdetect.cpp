//
// Created by yaione on 3/5/22.
//

#include "rmcv/objdetect.h"

namespace rm {

    void FindLightBars(std::vector<std::vector<cv::Point>> &input, std::vector<rm::LightBar> &output, float minRatio,
                       float maxRatio, float tilAngle, float minArea, rm::CampType camp) {
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

    void FindLightBars(vector<std::vector<cv::Point>> &contours, vector<rm::LightBar> &lightBars, float minRatio,
                       float maxRatio, float tiltAngle, float minArea, float maxArea, cv::Mat &frame,
                       bool useFitEllipse) {
        if (contours.size() < 2) return;
        lightBars.clear();

        for (auto &contour: contours) {
            if (contour.size() < 6 || cv::contourArea(contour) < minArea || cv::contourArea(contour) > maxArea)
                continue;

            cv::RotatedRect ellipse = cv::fitEllipseDirect(contour);
            cv::RotatedRect box = useFitEllipse ? cv::fitEllipseDirect(contour) : cv::minAreaRect(contour);

            // Aspect ratio
            float ratio = std::max(box.size.width, box.size.height) / std::min(box.size.width, box.size.height);
            if (ratio > maxRatio || ratio < minRatio) continue;

            // Angle (Perpendicular to the frame is considered as 90 degrees, left < 90, right > 90)
            float angle = ellipse.angle > 90 ? ellipse.angle - 90 : ellipse.angle + 90;
            if (abs(angle - 90) > tiltAngle) continue;

            cv::Mat ROI = frame(cv::boundingRect(contour));
            cv::Scalar meanValue = cv::mean(ROI);

            lightBars.emplace_back(box, angle, meanValue.val[0] > meanValue.val[2] ? rm::CAMP_BLUE : rm::CAMP_RED);
        }
    }

    void
    FindArmour(std::vector<rm::LightBar> &input, std::vector<rm::Armour> &output, float maxAngleDif, float errAngle,
               float minBoxRatio, float maxBoxRatio, float lenRatio, rm::CampType ownCamp, cv::Size frameSize) {
        output.clear();
        if (input.size() < 2) return;

        //
        std::sort(input.begin(), input.end(), [](const rm::LightBar &lightBar1, const rm::LightBar &lightBar2) {
            return lightBar1.center.x < lightBar2.center.x;
        });

        float yDifHistory = -1;
        int lastJ = -1;

        for (int i = 0; i < input.size() - 1; i++) {
            if (input[i].camp == ownCamp) continue;
            for (int j = i + 1; j < input.size(); j++) {
                if (input[j].camp == ownCamp) continue;

                // TODO: abstract into a function.
                bool conflict = false;
                for (int k = 0; k < input.size(); k++) {
                    if (k != i && k != j) {
                        if (input[k].center.x > min(input[i].center.x, input[j].center.x) &&
                            input[k].center.x < max(input[i].center.x, input[j].center.x) &&
                            input[k].center.y > min((float) input[i].vertices[1].y, (float) input[j].vertices[1].y) &&
                            input[k].center.y < max((float) input[i].vertices[0].y, (float) input[j].vertices[0].y)) {
                            conflict = true;
                        }
                    }
                }
                if (conflict)continue;

                // Poor angle between two light bar
                float angleDif = abs(input[i].angle - input[j].angle);
                if (angleDif > maxAngleDif) continue;

                float y = abs(input[i].center.y - input[j].center.y);
                float x = abs(input[i].center.x - input[j].center.x);
                float angle = atan2(y, x) * 180.0f / (float) CV_PI;
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

                float compensate = sin(atan2(min(heightI, heightJ), max(heightI, heightJ)));
                float distance = rm::PointDistance(input[i].center, input[j].center);
                float boxRatio = (max(heightI, heightI)) / (distance / compensate);
                if (boxRatio > maxBoxRatio || boxRatio < minBoxRatio)continue;

                if (abs(input[i].center.y - input[j].center.y) > ((input[i].size.height + input[j].size.height) / 2))
                    continue;

                cv::Point centerArmour((int) ((input[i].center.x + input[j].center.x) / 2.0f),
                                       (int) ((input[i].center.y + input[j].center.y) / 2.0f));
                cv::Point centerFrame(frameSize.width / 2, frameSize.height / 2);

                float yDiff = min(input[j].size.height, input[i].size.height);
                if (lastJ == i) {
                    if (yDifHistory < yDiff) {
                        output.pop_back();
                    } else {
                        continue;
                    }
                }
                yDifHistory = yDiff;
                lastJ = j;

                output.push_back(
                        rm::Armour({input[i], input[j]}, input[i].camp, rm::PointDistance(centerArmour, centerFrame)));
            }
        }

        if (!output.empty()) {
            std::sort(output.begin(), output.end(), [](const rm::Armour &armour1, const rm::Armour &armour2) {
                return armour1.distance2D < armour2.distance2D;
            });
        }
    }

    void SolveAirTrack(rm::Armour &target, double g, double v0, double hOffset, float motorAngle) {
        double h = (target.tvecs.ptr<double>(0)[1] - hOffset) / 100;
        double d = target.tvecs.ptr<double>(0)[2] / 100;

        double dPitch = -atan2(h, d) + motorAngle;               // Horizontal related angle.
        double hWorld = d * tan(dPitch);                            // Ground related height.
        double shootTheta = rm::NewtonIteration(rm::ProjectileMotionFD, {g, d, hWorld, v0});

        target.pitch = (float) shootTheta - motorAngle;
        target.yaw = (float) atan2(target.tvecs.ptr<double>(0)[0], target.tvecs.ptr<double>(0)[2]);
        target.airTime = d / v0 * cos(shootTheta);
    }

    void SolveAirTrack(Armour &target, double v0, double hOffset, double wOffset) {
        target.pitch = (float) atan2(target.tvecs.ptr<double>(0)[1] - hOffset, target.tvecs.ptr<double>(0)[2]);
        target.yaw = (float) atan2(target.tvecs.ptr<double>(0)[0] - wOffset, target.tvecs.ptr<double>(0)[2]);
        target.airTime = target.tvecs.ptr<double>(0)[2] / v0;
    }

    void SolveArmourPose(rm::Armour &target, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Size2f exactSize) {
        target.rvecs = cv::Mat::zeros(3, 1, CV_64FC1);
        target.tvecs = cv::Mat::zeros(3, 1, CV_64FC1);

        std::vector<cv::Point3f> exactPoint{cv::Point3f(-exactSize.width / 2.0f, exactSize.height / 2.0f, 0),
                                            cv::Point3f(exactSize.width / 2.0f, exactSize.height / 2.0f, 0),
                                            cv::Point3f(exactSize.width / 2.0f, -exactSize.height / 2.0f, 0),
                                            cv::Point3f(-exactSize.width / 2.0f, -exactSize.height / 2.0f, 0)};

        std::vector<cv::Point2f> tdCoordinate{target.vertices[1], target.vertices[2], target.vertices[3],
                                              target.vertices[0]};

        cv::solvePnP(exactPoint, tdCoordinate, cameraMatrix, distCoeffs, target.rvecs, target.tvecs, false,
                     cv::SOLVEPNP_ITERATIVE);
    }

    void SolveCameraPose(cv::Mat &rvecs, cv::Mat &tvecs, cv::Mat &output) {
        output = cv::Mat::zeros(3, 1, CV_64FC1);

        cv::Mat rotT = cv::Mat::eye(3, 3, CV_64F);
        cv::Rodrigues(tvecs, rotT);

        double rm[9];
        cv::Mat rotM(3, 3, CV_64FC1, rm);
        cv::Rodrigues(rvecs, rotM);
        double r11 = rotM.ptr<double>(0)[0];
        double r12 = rotM.ptr<double>(0)[1];
        double r13 = rotM.ptr<double>(0)[2];
        double r21 = rotM.ptr<double>(1)[0];
        double r22 = rotM.ptr<double>(1)[1];
        double r23 = rotM.ptr<double>(1)[2];
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

        output.ptr<float>(0)[0] = (float) thetaX * -1;
        output.ptr<float>(0)[1] = (float) thetaY * -1;
        output.ptr<float>(0)[2] = (float) thetaZ * -1;
    }

}
