//
// Created by yaione on 3/3/2022.
//

#ifndef RMCV_UTILS_H
#define RMCV_UTILS_H

#include <cmath>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <dirent.h>
#include <filesystem>
#include <cstring>
#include "opencv2/opencv.hpp"

namespace rm {
    enum RectType {
        RECT_TALL = 0, RECT_SIDE = 1
    };

    enum FileType {
        FILETYPE_REGULAR_FILE = 0,
        FILETYPE_DIRECTORY = 1,
        FILETYPE_SYMBOLIC_LINK = 2,
        FILETYPE_SOCKET = 3,
        FILETYPE_UNKNOWN = -1
    };

    rm::FileType GetFileType(const char *filename);

    bool ListFiles(const char *path, std::vector<std::string> &filenames);

    cv::Rect
    GetROI(cv::Point2f *imagePoints, int pointsCount, float scaleFactor = 1.0f, const cv::Size &frameSize = {-1, -1},
           const cv::Rect &previous = {0, 0, 0, 0});

    cv::Rect GetROI(cv::Point2f *imagePoints, int pointsCount, const cv::Size2f &scaleFactor = {1, 1},
                    const cv::Size &frameSize = {-1, -1}, const cv::Rect &previous = {0, 0, 0, 0});

    /// Reorder vertices of a rectangle to match the point order in left down, left up, right up, right down.
    /// \param input The original rectangle.
    /// \param output Reordered vertices.
    /// \param type Rectangle type.
    void reorder_vertices(cv::RotatedRect &input, cv::Point2f *output, RectType type);

    /// Calibrate given point sets in to specified aspect ratio.
    /// \param input Origin point sets.
    /// \param output Points sets in specified aspect ratio.
    /// \param outRatio Aspect ratio.
    void CalcPerspective(cv::Point2f input[4], cv::Point2f output[4], float outRatio = 1.0f);

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

    /// The f(x)/f'(x) function of projectile motion.
    /// \param theta The angle of the initial shot of the oblique throwing motion, in radians.
    /// \param literals The independent variables, in the order of g, d, h, v0.
    /// \return f(x)/f'(x)
    double ProjectileMotionFD(double theta, std::vector<double> literals);



    /// Return the distance between two given points.
    /// \param pt1 First point.
    /// \param pt2 Second point.
    /// \return Distance.
    float PointDistance(const cv::Point2f &pt1, const cv::Point2f &pt2);

    /// Return the distance between two given points.
    /// \param pt1 First point.
    /// \param pt2 Second point.
    /// \return Distance.
    float PointDistance(const cv::Point2i &pt1, const cv::Point2i &pt2);

    /// Return the midpoint between two given points.
    /// \param pt1 First point.
    /// \param pt2 Second point.
    /// \return Midpoint.
    cv::Point2f LineCenter(const cv::Point2f &pt1, const cv::Point2f &pt2);

    /// Expand the cord by the given length while center of the cord remain still.
    /// \param pt1 First point of the cord.
    /// \param pt2 Second point of the cord.
    /// \param deltaLen The length to be expanded.
    /// \param dst1 The first point after expanding.
    /// \param dst2 The second point after expanding.
    void
    ExtendCord(const cv::Point2f &pt1, const cv::Point2f &pt2, float deltaLen, cv::Point2f &dst1, cv::Point2f &dst2);

    std::string PathCombine(const std::string &path1, const std::string &path2);

    std::string int2str(int number);
}

#endif //RMCV_UTILS_H
