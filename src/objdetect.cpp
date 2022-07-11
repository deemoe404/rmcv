//
// Created by yaione on 3/5/22.
//

#include "include/objdetect.h"

namespace rm {
    bool MatchLightBlob(const std::vector<cv::Point> &contour, float minRatio, float maxRatio, float tiltAngle,
                        float minArea, float maxArea, cv::RotatedRect &lightBlobBox, bool fitEllipse) {
        if (contour.size() < 6 || cv::contourArea(contour) < minArea || cv::contourArea(contour) > maxArea)
            return false;

        cv::RotatedRect ellipse = cv::fitEllipseDirect(contour); // Calculate tilt angle using ellipse anyway
        cv::RotatedRect box = fitEllipse ? ellipse : cv::minAreaRect(contour);

        // Aspect ratio
        float ratio = std::max(box.size.width, box.size.height) / std::min(box.size.width, box.size.height);
        if (ratio > maxRatio || ratio < minRatio) return false;

        // Tilt angle (Perpendicular to the frame is considered as 90 degrees, left < 90, right > 90)
        float angle = ellipse.angle > 90 ? ellipse.angle - 90 : ellipse.angle + 90;
        if (abs(angle - 90) > tiltAngle) return false;

        lightBlobBox = box;
        return true;
    }

    void FindLightBlobs(std::vector<rm::Contour> &contours, std::vector<rm::LightBlob> &lightBlobs, float minRatio,
                        float maxRatio, float tiltAngle, float minArea, float maxArea, const cv::Mat &source,
                        bool fitEllipse) {
        lightBlobs.clear();
        if (source.channels() != 3) return;

        cv::RotatedRect box;
        for (auto &contour: contours) {
            if (!rm::MatchLightBlob(contour, minRatio, maxRatio, tiltAngle, minArea, maxArea, box, fitEllipse))
                continue;

            cv::Scalar meanValue = cv::mean(source(cv::boundingRect(contour)));
            if (meanValue.val[1] > meanValue.val[0] && meanValue.val[1] > meanValue.val[2]) {
                lightBlobs.emplace_back(box, rm::CAMP_GUIDELIGHT);
            } else {
                lightBlobs.emplace_back(box, meanValue.val[0] > meanValue.val[2] ? rm::CAMP_BLUE : rm::CAMP_RED);
            }
        }
    }

    void FindLightBlobs(std::vector<rm::Contour> &contours, std::vector<rm::LightBlob> &lightBlobs, float minRatio,
                        float maxRatio, float tiltAngle, float minArea, float maxArea, rm::CampType camp,
                        bool fitEllipse) {
        lightBlobs.clear();

        cv::RotatedRect box;
        for (auto &contour: contours) {
            if (!rm::MatchLightBlob(contour, minRatio, maxRatio, tiltAngle, minArea, maxArea, box, fitEllipse))
                continue;

            lightBlobs.emplace_back(box, camp);
        }
    }

    bool LightBlobOverlap(std::vector<rm::LightBlob> &lightBlobs, int leftIndex, int rightIndex) {
        if (leftIndex < 0 || rightIndex > lightBlobs.size() || rightIndex - leftIndex < 2) return false;

        float lowerY = min(
                min((float) lightBlobs[leftIndex].vertices[1].y, (float) lightBlobs[leftIndex].vertices[2].y),
                min((float) lightBlobs[rightIndex].vertices[1].y, (float) lightBlobs[rightIndex].vertices[2].y));
        float UpperY = max(
                max((float) lightBlobs[leftIndex].vertices[0].y, (float) lightBlobs[leftIndex].vertices[3].y),
                max((float) lightBlobs[rightIndex].vertices[0].y, (float) lightBlobs[rightIndex].vertices[3].y));

        for (int i = leftIndex; i < rightIndex; i++) {
            if (lightBlobs[i].center.x > lightBlobs[leftIndex].center.x &&
                lightBlobs[i].center.x < lightBlobs[rightIndex].center.x && lightBlobs[i].center.y > lowerY &&
                lightBlobs[i].center.y < UpperY) {
                return true;
            }
        }
        return false;
    }

    void FindArmour(std::vector<rm::LightBlob> &lightBlobs, std::vector<rm::Armour> &armours, float maxAngleDif,
                    float errAngle, float minBoxRatio, float maxBoxRatio, float lenRatio, rm::CampType enemy,
                    bool filter) {
        armours.clear();
        if (lightBlobs.size() < 2) return;

        // sort ascending by x
        std::sort(lightBlobs.begin(), lightBlobs.end(),
                  [](const rm::LightBlob &lightBlob1, const rm::LightBlob &lightBlob2) {
                      return lightBlob1.center.x < lightBlob2.center.x;
                  });

        float yDifHistory = -1;
        int lastJ = -1;

        for (int i = 0; i < lightBlobs.size() - 1; ++i) {
            if (lightBlobs[i].camp != enemy) continue;
            for (int j = i + 1; j < lightBlobs.size(); ++j) {
                if (lightBlobs[j].camp != enemy || rm::LightBlobOverlap(lightBlobs, i, j))
                    continue;

                // Poor angle between two light blobs
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

                if (filter) {
                    float yDiff = min(lightBlobs[j].size.height, lightBlobs[i].size.height);
                    if (lastJ == i) {
                        if (yDifHistory < yDiff) {
                            armours.pop_back();
                        } else {
                            continue;
                        }
                    }
                    yDifHistory = yDiff;
                    lastJ = j;
                }
                armours.push_back(rm::Armour({lightBlobs[i], lightBlobs[j]}, 0, lightBlobs[i].camp));
            }
        }
    }

}
