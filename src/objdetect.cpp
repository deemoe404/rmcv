//
// Created by yaione on 3/5/22.
//

#include "rmcv/objdetect.h"

namespace rm {
    bool
    JudgeLightBlob(const std::vector<cv::Point> &contour, float minRatio, float maxRatio, float tilAngle, float minArea,
                   float maxArea, bool useFitEllipse) {
        if (contour.size() < 6 || cv::contourArea(contour) < minArea || cv::contourArea(contour) > maxArea)
            return false;

        cv::RotatedRect ellipse = cv::fitEllipseDirect(contour);
        cv::RotatedRect box = useFitEllipse ? ellipse : cv::minAreaRect(contour);

        // Aspect ratio
        float ratio = std::max(box.size.width, box.size.height) / std::min(box.size.width, box.size.height);
        if (ratio > maxRatio || ratio < minRatio) return false;

        // Angle (Perpendicular to the frame is considered as 90 degrees, left < 90, right > 90)
        float angle = ellipse.angle > 90 ? ellipse.angle - 90 : ellipse.angle + 90;
        if (abs(angle - 90) > tilAngle) return false;

        return true;
    }

    void
    FindLightBlobs(std::vector<std::vector<cv::Point>> &contours, std::vector<rm::LightBlob> &lightBlobs, float minRatio,
                   float maxRatio, float tiltAngle, float minArea, float maxArea, rm::CampType camp,
                   bool useFitEllipse) {
        if (contours.size() < 2) return;
        lightBlobs.clear();

        for (auto &contour: contours) {
            if (!rm::JudgeLightBlob(contour, minRatio, maxRatio, tiltAngle, minArea, maxArea)) continue;

            lightBlobs.emplace_back(useFitEllipse ? cv::fitEllipseDirect(contour) : cv::minAreaRect(contour), camp);
        }
    }

    void FindLightBlobs(vector<std::vector<cv::Point>> &contours, vector<rm::LightBlob> &lightBlobs, float minRatio,
                        float maxRatio, float tiltAngle, float minArea, float maxArea, cv::Mat &frame,
                        bool useFitEllipse) {
        lightBlobs.clear();

        for (auto &contour: contours) {
            if (!rm::JudgeLightBlob(contour, minRatio, maxRatio, tiltAngle, minArea, maxArea)) continue;

            cv::Scalar meanValue = cv::mean(frame(cv::boundingRect(contour)));

            if (meanValue.val[1] > meanValue.val[0] && meanValue.val[1] > meanValue.val[2]) {
                lightBlobs.emplace_back(useFitEllipse ? cv::fitEllipseDirect(contour) : cv::minAreaRect(contour),
                                       rm::CAMP_OUTPOST);
            } else {
                lightBlobs.emplace_back(useFitEllipse ? cv::fitEllipseDirect(contour) : cv::minAreaRect(contour),
                                       meanValue.val[0] > meanValue.val[2] ? rm::CAMP_BLUE : rm::CAMP_RED);
            }
        }
    }

    void FindLightBlobs(cv::Mat &binary, std::vector<rm::LightBlob> &lightBlobs, float minRatio, float maxRatio,
                        float tiltAngle, float minArea, float maxArea, cv::Mat &frame, bool useFitEllipse) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        rm::FindLightBlobs(contours, lightBlobs, minRatio, maxRatio, tiltAngle, minArea, maxArea, frame, useFitEllipse);
    }

    bool LightBlobInterference(std::vector<rm::LightBlob> &lightBlobs, int leftIndex, int rightIndex) {
        for (int k = 0; k < lightBlobs.size(); k++) {
            if (k == leftIndex && k == rightIndex) continue;
            if (lightBlobs[k].center.x > min(lightBlobs[leftIndex].center.x, lightBlobs[rightIndex].center.x) &&
                lightBlobs[k].center.x < max(lightBlobs[leftIndex].center.x, lightBlobs[rightIndex].center.x) &&
                lightBlobs[k].center.y >
                min((float) lightBlobs[leftIndex].vertices[1].y, (float) lightBlobs[rightIndex].vertices[1].y) &&
                lightBlobs[k].center.y <
                max((float) lightBlobs[leftIndex].vertices[0].y, (float) lightBlobs[rightIndex].vertices[0].y)) {
                return true;
            }
        }
        return false;
    }

    void FindArmour(std::vector<rm::LightBlob> &lightBlobs, std::vector<rm::Armour> &armours, float maxAngleDif,
                    float errAngle, float minBoxRatio, float maxBoxRatio, float lenRatio, rm::CampType ownCamp,
                    cv::Size2f frameSize, bool filter) {
        cv::Point2f center = {frameSize.width / 2.0f, frameSize.height / 2.0f};

        rm::FindArmour(lightBlobs, armours, maxAngleDif, errAngle, minBoxRatio, maxBoxRatio, lenRatio, ownCamp, center,
                       filter);
    }

    void FindArmour(std::vector<rm::LightBlob> &lightBlobs, std::vector<rm::Armour> &armours, float maxAngleDif,
                    float errAngle, float minBoxRatio, float maxBoxRatio, float lenRatio, rm::CampType ownCamp,
                    cv::Point2f attention, bool filter) {
        armours.clear();
        if (lightBlobs.size() < 2) return;

        // sort vertices ascending by x
        std::sort(lightBlobs.begin(), lightBlobs.end(), [](const rm::LightBlob &lightBar1, const rm::LightBlob &lightBar2) {
            return lightBar1.center.x < lightBar2.center.x;
        });

        float yDifHistory = -1;
        int lastJ = -1;

        for (int i = 0; i < lightBlobs.size() - 1; i++) {
            if (lightBlobs[i].camp == ownCamp) continue;
            for (int j = i + 1; j < lightBlobs.size(); j++) {
                if (lightBlobs[j].camp == ownCamp) continue;
                if (rm::LightBlobInterference(lightBlobs, i, j))continue;

                // Poor angle between two light blob
                float angleDif = abs(lightBlobs[i].angle - lightBlobs[j].angle);
                if (angleDif > maxAngleDif) continue;

                float y = abs(lightBlobs[i].center.y - lightBlobs[j].center.y);
                float x = abs(lightBlobs[i].center.x - lightBlobs[j].center.x);
                float angle = atan2(y, x) * 180.0f / (float) CV_PI;
                float errorI = abs(lightBlobs[i].angle > 90 ? abs(lightBlobs[i].angle - angle) - 90 :
                                   abs(180 - lightBlobs[i].angle - angle) - 90);
                float errorJ = abs(lightBlobs[j].angle > 90 ? abs(lightBlobs[j].angle - angle) - 90 :
                                   abs(180 - lightBlobs[j].angle - angle) - 90);
                if (errorI > errAngle || errorJ > errAngle) continue;

                float heightI = lightBlobs[i].size.height;
                float heightJ = lightBlobs[j].size.height;
                float ratio = std::min(heightI, heightJ) / std::max(heightI, heightJ);
                if (ratio < lenRatio) continue;

                float compensate = sin(atan2(min(heightI, heightJ), max(heightI, heightJ)));
                float distance = rm::PointDistance(lightBlobs[i].center, lightBlobs[j].center);
                float boxRatio = (max(heightI, heightI)) / (distance / compensate);
                if (boxRatio > maxBoxRatio || boxRatio < minBoxRatio)continue;

                if (abs(lightBlobs[i].center.y - lightBlobs[j].center.y) >
                    ((lightBlobs[i].size.height + lightBlobs[j].size.height) / 2))
                    continue;

                cv::Point2f centerArmour((lightBlobs[i].center.x + lightBlobs[j].center.x) / 2.0f,
                                         (lightBlobs[i].center.y + lightBlobs[j].center.y) / 2.0f);

                float yDiff = min(lightBlobs[j].size.height, lightBlobs[i].size.height);
                if (lastJ == i) {
                    if (yDifHistory < yDiff) {
                        armours.pop_back();
                    } else {
                        if (filter) continue;
                    }
                }
                yDifHistory = yDiff;
                lastJ = j;

                armours.push_back(rm::Armour({lightBlobs[i], lightBlobs[j]}, rm::PointDistance(centerArmour, attention),
                                             lightBlobs[i].camp));
            }
        }

        if (armours.size() > 1) {
            std::sort(armours.begin(), armours.end(), [](const rm::Armour &armour1, const rm::Armour &armour2) {
                return armour1.rank < armour2.rank;
            });
        }
    }

    void
    SolveShootFactor(cv::Mat &translationVector, rm::ShootFactor &shootFactor, double g, double v0, double deltaHeight,
                     cv::Point2f offset, rm::CompensateMode mode) {
        float yaw = (float) atan2(translationVector.ptr<double>(0)[0] - offset.x, translationVector.ptr<double>(0)[2]) *
                    180.0f / (float) CV_PI;
        float pitch = 0;
        double airTime = 0;

        double d = (translationVector.ptr<double>(0)[2]) / 100.0;

        if (mode == rm::COMPENSATE_NONE) {
            pitch = (float) atan2(translationVector.ptr<double>(0)[1] - offset.y, translationVector.ptr<double>(0)[2]) *
                    180.0f / (float) CV_PI;
            airTime = d / v0;
        } else if (mode == rm::COMPENSATE_CLASSIC) {
            double normalAngle = atan2(deltaHeight / 100.0, d) * 180.0 / CV_PI;
            double centerAngle =
                    -atan2(translationVector.ptr<double>(0)[1] - offset.y, translationVector.ptr<double>(0)[2]) *
                    180.0 / CV_PI;
            double targetAngle = rm::ProjectileAngle(v0, g, d, deltaHeight / 100.0) * 180.0 / CV_PI;

            pitch = (float) ((centerAngle - normalAngle) + targetAngle);
            airTime = d / abs(v0 * cos(targetAngle));
        } else if (mode == rm::COMPENSATE_NI) {
            //TODO: Fix the bug of NI first!!!!
        }

        shootFactor.pitchAngle = pitch;
        shootFactor.yawAngle = yaw;
        shootFactor.estimateAirTime = airTime;
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
