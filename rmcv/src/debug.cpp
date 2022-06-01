//
// Created by yaione on 3/17/22.
//

#include "include/debug.h"

namespace rm::debug {
    void DrawArmours(const std::vector<rm::Armour> &input, cv::Mat &output, int index) {
        if (input.empty()) return;

        std::vector<std::vector<cv::Point>> contours;
        if (index < 0 || index > input.size()) {
            for (auto &armour: input) {
                std::vector<cv::Point> vertices;
                std::vector<cv::Point> icons;
                for (auto &vertex: armour.vertices) {
                    vertices.push_back(vertex);
                }
                for (auto &vertex: armour.icon) {
                    icons.push_back(vertex);
                }
                contours.push_back(vertices);
                contours.push_back(icons);
            }
        } else {
            std::vector<cv::Point> vertices;
            std::vector<cv::Point> icons;
            for (auto &vertex: input[index].vertices) {
                vertices.push_back(vertex);
            }
            for (auto &vertex: input[index].icon) {
                icons.push_back(vertex);
            }
            contours.push_back(vertices);
            contours.push_back(icons);
        }
        cv::drawContours(output, contours, -1, {0, 255, 255}, 1);
    }

    void DrawArmour(rm::Armour input, Mat &output) {
        std::vector<std::vector<cv::Point>> contours;

        std::vector<cv::Point> vertices;
        std::vector<cv::Point> icons;
        for (auto &vertex: input.vertices) {
            vertices.push_back(vertex);
        }
        for (auto &vertex: input.icon) {
            icons.push_back(vertex);
        }
        contours.push_back(vertices);
        contours.push_back(icons);

        cv::drawContours(output, contours, -1, {0, 255, 255}, 1);
    }

    void DrawLightBlobs(const vector<rm::LightBlob> &input, Mat &output, int index) {
        if (input.empty()) return;

        std::vector<std::vector<cv::Point>> contoursRed;
        std::vector<std::vector<cv::Point>> contoursBlue;
        std::vector<std::vector<cv::Point>> contoursOutpost;

        if (index < 0 || index > input.size()) {
            for (auto &lightBar: input) {
                std::vector<cv::Point> vertices;
                for (auto &vertex: lightBar.vertices) {
                    vertices.push_back(vertex);
                }
                if (lightBar.camp == rm::CAMP_RED) {
                    contoursRed.push_back(vertices);
                } else if (lightBar.camp == rm::CAMP_BLUE) {
                    contoursBlue.push_back(vertices);
                } else if (lightBar.camp == rm::CAMP_GUIDELIGHT) {
                    contoursOutpost.push_back(vertices);
                }
            }
        } else {
            std::vector<cv::Point> vertices;
            for (auto &vertex: input[index].vertices) {
                vertices.push_back(vertex);
            }
            if (input[index].camp == rm::CAMP_RED) {
                contoursRed.push_back(vertices);
            } else if (input[index].camp == rm::CAMP_BLUE) {
                contoursBlue.push_back(vertices);
            } else if (input[index].camp == rm::CAMP_GUIDELIGHT) {
                contoursOutpost.push_back(vertices);
            }
        }

        cv::drawContours(output, contoursRed, -1, {0, 0, 255}, 1);
        cv::drawContours(output, contoursBlue, -1, {255, 0, 0}, 1);
        cv::drawContours(output, contoursOutpost, -1, {0, 255, 0}, 1);
    }

    void PrintMat(cv::Mat &input, int decimal) {
        for (int i = 0; i < input.size().height; i++) {
            std::cout << "[ ";
            for (int j = 0; j < input.size().width; j++) {
                std::cout << std::fixed << std::setw(2) << std::setprecision(decimal) << input.at<float>(i, j);
                if (j != input.size().width - 1)
                    std::cout << ", ";
                else
                    std::cout << " ]" << std::endl;
            }
        }
    }
}
