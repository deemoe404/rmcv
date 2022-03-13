//
// Created by yaione on 3/3/2022.
//

#ifndef RM_STANDARD2022_UTILS_H
#define RM_STANDARD2022_UTILS_H

#include <cmath>
#include <vector>
#include "filesystem"
#include "opencv2/opencv.hpp"

namespace rm {
    enum RectType {
        RECT_TALL = 0, RECT_SIDE = 1
    };

    /// Reorder vertices of a rectangle to match the point order in left down, left up, right up, right down.
    /// \param input The original rectangle.
    /// \param output Reordered vertices.
    /// \param type Rectangle type.
    void VerticesRectify(cv::RotatedRect &input, cv::Point *output, RectType type);

    /// Use newton's iteration to approach the approx solve of function.
    /// \param fd The f(x)/f'(x) function of goal function.
    /// \param x0 x to start the iteration.
    /// \param error Maximum error before iteration stops.
    /// \param cycle Maximum cycle the iteration could run.
    /// \return The solve.
    double NewtonIteration(double (*fd)(double), double x0 = 0, double error = 0.0001, int cycle = 1024);

    /// Use newton's iteration to approach the approx solve of function.
    /// \param fd The f(x)/f'(x) function of goal function.
    /// \param literals The literals that all f(x) needed.
    /// \param x0 x to start the iteration.
    /// \param error Maximum error before iteration stops.
    /// \param cycle Maximum cycle the iteration could run.
    /// \return The solve.
    double
    NewtonIteration(double (*fd)(double, std::vector<double>), const std::vector<double> &literals, double x0 = 0,
                    double error = 0.0001, int cycle = 1024);
    // TODO: Add parameter of the order to the solve to accelerate the function.

    /// The f(x)/f'(x) function of projectile motion.
    /// \param theta The angle of the initial shot of the oblique throwing motion, in radians.
    /// \param literals The independent variables, in the order of g, d, h, v0.
    /// \return f(x)/f'(x)
    double ProjectileMotionFD(double theta, std::vector<double> literals);

    /// Return the distance between two given points.
    /// \param pt1 Point one.
    /// \param pt2 Point two.
    /// \return Distance.
    float PointDistance(cv::Point2f pt1, cv::Point2f pt2);
    // TODO: Point instead of Point2f?

    /// Expand the cord by the given length.
    /// \param pt1 First point of the cord.
    /// \param pt2 Second point of the cord.
    /// \param deltaLen The length to be expanded.
    /// \param dst1 The first point after expanding.
    /// \param dst2 The second point after expanding.
    void ExCord(cv::Point pt1, cv::Point pt2, int deltaLen, cv::Point &dst1, cv::Point &dst2);

    std::string PathCombine(const std::string& path1, const std::string& path2);

    std::string int2str(int number);
}

#endif //RM_STANDARD2022_UTILS_H
