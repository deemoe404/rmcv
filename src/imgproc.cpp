//
// Created by yaione on 2/26/2022.
//

#include "imgproc.h"

namespace rm
{
    void CalcRatio(cv::Mat& source, cv::Mat& calibration,
                   cv::Point2f vertices[4], cv::Size outSize)
    {
        // handel the case of vertices out of screen
        for (int i = 0; i < 4; i++)
        {
            if (vertices[i].x < 0 || vertices[i].y < 0 || vertices[i].x > (
                    float)
                source.cols - 1 || vertices[i].y > (float)source.rows - 1)
            {
                if (vertices[i].x < 0) vertices[i].x = 0;
                if (vertices[i].y < 0) vertices[i].y = 0;
                if (vertices[i].x > (float)source.cols - 1)
                    vertices[i].x = (
                        float)source.cols - 1;
                if (vertices[i].y > (float)source.rows - 1)
                    vertices[i].y = (
                        float)source.rows - 1;
            }
        }

        cv::Rect box = cv::boundingRect(std::vector<cv::Point>({
            vertices[0], vertices[1], vertices[2], vertices[3]
        }));
        cv::Point2f srcPts[3] = {
            {vertices[1].x - (float)box.x, vertices[1].y - (float)box.y},
            {vertices[2].x - (float)box.x, vertices[2].y - (float)box.y},
            {vertices[0].x - (float)box.x, vertices[0].y - (float)box.y}
        };
        cv::Point2f dstPts[3] = {
            {0, 0},
            {(float)(box.width), 0},
            {0, (float)(box.height)}
        };
        auto warp = cv::getAffineTransform(srcPts, dstPts);
        cv::warpAffine(source(box), calibration, warp, calibration.size());

        // resize to the dst size
        cv::resize(calibration, calibration, outSize);
    }

    void CalcGamma(cv::Mat& source, cv::Mat& calibration, float gamma)
    {
        cv::Mat lookUpTable(1, 256, CV_8U);
        uchar* p = lookUpTable.ptr();
        for (int i = 0; i < 256; ++i)
        {
            p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
        }

        // search for LUT table
        cv::LUT(source, lookUpTable, calibration);
    }

    void extract_color(cv::InputArray image, cv::OutputArray binary, color_camp enemy, int lower_bound)
    {
        std::vector<cv::Mat> channels;
        split(image, channels);

        if (enemy == CAMP_GUIDELIGHT)
        {
            auto gray = channels[1] - channels[2];
            inRange(gray, lower_bound, 255, binary);
        }
        else
        {
            auto gray = channels[enemy == CAMP_BLUE ? 0 : 2] - channels[enemy == CAMP_BLUE ? 2 : 0];
            inRange(gray, lower_bound, 255, binary);
        }
    }

    void AutoEnhance(cv::Mat& frame, float maxGainFactor, float minGainFactor)
    {
        cv::Scalar meanValue = cv::mean(frame);
        float meanC3 = (float)(meanValue[0] + meanValue[1] + meanValue[2]) /
            3.0f;
        float k = 2.0f / (maxGainFactor - minGainFactor);
        float b = 3.0f - maxGainFactor * k;
        float gammaFactor = k * meanC3 + b;

        // Map [1.0, -3.0] to [1.0, 0.0]
        if (gammaFactor <= 1.0f && gammaFactor >= -3.0f)
        {
            gammaFactor = 1.0f + (gammaFactor - 1) / 4.0f;
        }
        else if (gammaFactor < -3.0f)
        {
            // Frame too dark, gamma might not helpful in this case.
            gammaFactor = 0;
        }

        rm::CalcGamma(frame, frame, gammaFactor);
    }

    void AutoBinarize(cv::Mat& image, cv::Mat& binary)
    {
        cv::cvtColor(image, binary, cv::COLOR_BGR2GRAY);
        cv::Scalar meanValue = cv::mean(binary);
        cv::inRange(
            binary, cv::Scalar{meanValue[0], meanValue[1], meanValue[2]},
            cv::Scalar{255, 255, 255}, binary);

        binary.convertTo(binary, CV_32FC1, 1.0 / 255.0);
    }
}
