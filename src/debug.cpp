//
// Created by yaione on 3/17/22.
//

#include "debug.h"

namespace rm::debug
{
    void DrawArmours(const std::vector<rm::armour>& input, cv::Mat& output, int index)
    {
        if (input.empty()) return;

        std::vector<std::vector<cv::Point>> contours;
        if (index < 0 || index > input.size())
        {
            for (auto& armour : input)
            {
                std::vector<cv::Point> vertices;
                std::vector<cv::Point> icons;
                for (auto& vertex : armour.vertices)
                {
                    vertices.push_back(vertex);
                }
                for (auto& vertex : armour.icon)
                {
                    icons.push_back(vertex);
                }
                contours.push_back(vertices);
                contours.push_back(icons);
            }
        }
        else
        {
            std::vector<cv::Point> vertices;
            std::vector<cv::Point> icons;
            for (auto& vertex : input[index].vertices)
            {
                vertices.push_back(vertex);
            }
            for (auto& vertex : input[index].icon)
            {
                icons.push_back(vertex);
            }
            contours.push_back(vertices);
            contours.push_back(icons);
        }
        cv::drawContours(output, contours, -1, {0, 255, 255}, 1);
    }

    void DrawArmour(rm::armour input, cv::Mat& output)
    {
        std::vector<std::vector<cv::Point>> contours;

        std::vector<cv::Point> vertices;
        std::vector<cv::Point> icons;
        for (auto& vertex : input.vertices)
        {
            vertices.push_back(vertex);
        }
        for (auto& vertex : input.icon)
        {
            icons.push_back(vertex);
        }
        contours.push_back(vertices);
        contours.push_back(icons);

        cv::drawContours(output, contours, -1, {0, 255, 255}, 1);
    }

    void draw_lightblobs(const std::vector<lightblob>& positive, const std::vector<contour>& negative, cv::Mat& output,
                         const int index)
    {
        if (positive.empty() && negative.empty()) return;

        auto draw_lightblob = [&](const lightblob& lightblob)
        {
            const std::vector<cv::Point> vertices(std::begin(lightblob.vertices), std::end(lightblob.vertices));
            const std::vector<std::vector<cv::Point>> contours = {vertices};

            const cv::Scalar color = lightblob.target == CAMP_RED ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
            drawContours(output, contours, -1, color, 1);
        };

        if (index < 0 || index > positive.size())
            for (auto& lightblob : positive) draw_lightblob(lightblob);
        else
            draw_lightblob(positive[index]);

        if (!negative.empty())
            drawContours(output, negative, -1, {0, 255, 255}, 1);
    }

    void PrintMat(cv::Mat& input, int decimal)
    {
        for (int i = 0; i < input.size().height; i++)
        {
            std::cout << "[ ";
            for (int j = 0; j < input.size().width; j++)
            {
                std::cout << std::fixed << std::setw(2) << std::setprecision(decimal) << input.at<float>(i, j);
                if (j != input.size().width - 1)
                    std::cout << ", ";
                else
                    std::cout << " ]" << std::endl;
            }
        }
    }
}
