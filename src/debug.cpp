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
                for (auto &vertex: armour.vertices) {
                    vertices.push_back(vertex);
                }
                contours.push_back(vertices);
            }
        } else {
            std::vector<cv::Point> vertices;
            for (auto &vertex: input[index].vertices) {
                vertices.push_back(vertex);
            }
            contours.push_back(vertices);
        }
        cv::drawContours(output, contours, -1, {0, 255, 255}, 1);
    }

    void DrawLightBars(const vector<rm::LightBar> &input, Mat &output, int index) {
        if (input.empty()) return;

        std::vector<std::vector<cv::Point>> contours;
        if (index < 0 || index > input.size()) {
            for (auto &armour: input) {
                std::vector<cv::Point> vertices;
                for (auto &vertex: armour.vertices) {
                    vertices.push_back(vertex);
                }
                contours.push_back(vertices);
            }
        } else {
            std::vector<cv::Point> vertices;
            for (auto &vertex: input[index].vertices) {
                vertices.push_back(vertex);
            }
            contours.push_back(vertices);
        }
        cv::drawContours(output, contours, -1, {0, 0, 255}, 1);
    }
}
