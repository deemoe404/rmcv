//
// Created by yaione on 3/17/22.
//

#include "rmcv/debug.h"

namespace rm::debug {
    void DrawArmours(const std::vector<rm::Armour> &input, cv::Mat &output, int index) {
        if (input.empty()) return;

        std::vector<std::vector<cv::Point>> contours;
        if (index < 0 || index > input.size()) {
            for (auto &armour: input) {
                std::vector<cv::Point> vertices;
                for (auto &vertex: armour.icon) {
                    vertices.push_back(vertex);
                }
                contours.push_back(vertices);
            }
        } else {
            std::vector<cv::Point> vertices;
            for (auto &vertex: input[index].icon) {
                vertices.push_back(vertex);
            }
            contours.push_back(vertices);
        }
        cv::drawContours(output, contours, -1, {0, 255, 255}, 1);
    }

    void DrawLightBars(const vector<rm::LightBar> &input, Mat &output, int index) {
        if (input.empty()) return;

        std::vector<std::vector<cv::Point>> contoursRed;
        std::vector<std::vector<cv::Point>> contoursBlue;

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
            }
        }

        cv::drawContours(output, contoursRed, -1, {0, 0, 255}, 1);
        cv::drawContours(output, contoursBlue, -1, {255, 0, 0}, 1);
    }
}
