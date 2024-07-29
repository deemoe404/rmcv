//
// Created by yaione on 3/5/22.
//

#include "objdetect.h"

namespace rm
{
    bool MatchLightBlob(const std::vector<cv::Point>& contour, float minRatio, float maxRatio, float tiltAngle,
                        float minArea, float maxArea, cv::RotatedRect& lightBlobBox, bool fitEllipse)
    {
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

    void FindLightBlobs(std::vector<contour>& contours, std::vector<lightblob>& lightBlobs, float minRatio,
                        float maxRatio, float tiltAngle, float minArea, float maxArea, const cv::Mat& source,
                        bool fitEllipse)
    {
        lightBlobs.clear();
        if (source.channels() != 3) return;

        cv::RotatedRect box;
        for (auto& contour : contours)
        {
            if (!MatchLightBlob(contour, minRatio, maxRatio, tiltAngle, minArea, maxArea, box, fitEllipse))
                continue;

            cv::Scalar meanValue = mean(source(boundingRect(contour)));
            if (meanValue.val[1] > meanValue.val[0] && meanValue.val[1] > meanValue.val[2])
            {
                lightBlobs.emplace_back(box, CAMP_GUIDELIGHT);
            }
            else
            {
                lightBlobs.emplace_back(box, meanValue.val[0] > meanValue.val[2] ? CAMP_BLUE : CAMP_RED);
            }
        }
    }

    auto filter_lightblobs(const std::vector<contour>& contours, const float tilt_max, const range<float> ratio_range,
                           const range<double> area_range, camp enemy)
        -> std::tuple<std::vector<lightblob>, std::vector<contour>>
    {
        std::vector<lightblob> positive;
        std::vector<contour> negative;

        for (auto& contour : contours)
        {
            if (contour.size() < 6 || !area_range.contains(contourArea(contour)))
                continue;

            bool negative_flag = false;
            cv::RotatedRect ellipse = fitEllipseDirect(contour);
            cv::RotatedRect box = minAreaRect(contour);

            if (const float ratio =
                    std::max(ellipse.size.width, ellipse.size.height) /
                    std::min(ellipse.size.width, ellipse.size.height);
                !ratio_range.contains(ratio))
                negative_flag = true;

            // Perpendicular to the frame is considered as 90 degrees, tilt left < 90, tilt right > 90
            if (const float angle = ellipse.angle > 90 ? ellipse.angle - 90 : ellipse.angle + 90;
                abs(angle - 90) > tilt_max)
                negative_flag = true;

            if (negative_flag) negative.push_back(contour);
            else positive.emplace_back(ellipse, enemy);
        }

        return {positive, negative};
    }

    bool LightBlobOverlap(const std::vector<lightblob>& lightBlobs, const int leftIndex, const int rightIndex)
    {
        if (leftIndex < 0 || rightIndex > lightBlobs.size() || rightIndex - leftIndex < 2) return false;
        if (lightBlobs[leftIndex].target != lightBlobs[rightIndex].target) return false;

        float lowerY = std::min(
            std::min((float)lightBlobs[leftIndex].vertices[1].y, (float)lightBlobs[leftIndex].vertices[2].y),
            std::min((float)lightBlobs[rightIndex].vertices[1].y, (float)lightBlobs[rightIndex].vertices[2].y));
        float UpperY = std::max(
            std::max((float)lightBlobs[leftIndex].vertices[0].y, (float)lightBlobs[leftIndex].vertices[3].y),
            std::max((float)lightBlobs[rightIndex].vertices[0].y, (float)lightBlobs[rightIndex].vertices[3].y));

        for (int i = leftIndex; i < rightIndex; i++)
        {
            if (lightBlobs[i].target != lightBlobs[leftIndex].target) continue;
            if (lightBlobs[i].center.x > lightBlobs[leftIndex].center.x &&
                lightBlobs[i].center.x < lightBlobs[rightIndex].center.x && lightBlobs[i].center.y > lowerY &&
                lightBlobs[i].center.y < UpperY)
            {
                return true;
            }
        }
        return false;
    }

    void filter_armours(std::vector<lightblob>& lightblobs, std::vector<armour>& armours,
                        const float angle_difference_max, const float shear_max, const float lenght_ratio_max,
                        const camp enemy)
    {
        armours.clear();
        if (lightblobs.size() < 2) return;

        for (int i = 0; i < lightblobs.size() - 1; ++i)
        {
            if (lightblobs[i].target != enemy)
                continue;
            for (int j = i + 1; j < lightblobs.size(); ++j)
            {
                if (lightblobs[j].target != enemy)
                    continue;

                if (const float angle_difference = abs(lightblobs[i].angle - lightblobs[j].angle);
                    angle_difference > angle_difference_max)
                    continue;

                const float y = abs(lightblobs[i].center.y - lightblobs[j].center.y);
                const float x = abs(lightblobs[i].center.x - lightblobs[j].center.x);
                const float rect_angle = atan2(y, x) * 180.0f / static_cast<float>(CV_PI);
                const float shear_i = abs(lightblobs[i].angle > 90
                                              ? abs(lightblobs[i].angle - rect_angle) - 90
                                              : abs(180 - lightblobs[i].angle - rect_angle) - 90);
                const float shear_j = abs(lightblobs[j].angle > 90
                                              ? abs(lightblobs[j].angle - rect_angle) - 90
                                              : abs(180 - lightblobs[j].angle - rect_angle) - 90);
                if (shear_i > shear_max || shear_j > shear_max)
                    continue;

                float height_i = lightblobs[i].size.height;
                float height_j = lightblobs[j].size.height;
                if (const float ratio = std::min(height_i, height_j) / std::max(height_i, height_j);
                    ratio < lenght_ratio_max)
                    continue;

                if (abs(lightblobs[i].center.y - lightblobs[j].center.y) >
                    (lightblobs[i].size.height + lightblobs[j].size.height) / 2)
                    continue;

                if (abs(lightblobs[i].center.x - lightblobs[j].center.x) >
                    (lightblobs[i].size.height + lightblobs[j].size.height) * 2)
                    continue;

                armours.push_back(armour({lightblobs[i], lightblobs[j]}, 0, lightblobs[i].target));
            }
        }
    }
}
