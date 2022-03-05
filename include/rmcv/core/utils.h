//
// Created by yaione on 3/3/2022.
//

#ifndef RM_STANDARD2022_UTILS_H
#define RM_STANDARD2022_UTILS_H

#include <cmath>
#include <vector>
#include "opencv2/opencv.hpp"

namespace rm {
    enum RectType {
        RECT_TALL = 0, RECT_SIDE = 1
    };

    double NewtonIteration(double (*fd)(double), double x0 = 0, double error = 0.001, int cycle = 1024);

    double
    NewtonIteration(double (*fd)(double, std::vector<double>), const std::vector<double> &literals, double x0 = 0,
                    double error = 0.001, int cycle = 1024);

    void VerticesRectify(cv::RotatedRect &input, cv::Point2f *output, RectType type);
}

#endif //RM_STANDARD2022_UTILS_H
