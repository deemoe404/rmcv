//
// Created by yaione on 3/3/2022.
//

#ifndef RM_STANDARD2022_UTILS_H
#define RM_STANDARD2022_UTILS_H

#include <cmath>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <dirent.h>
#include <filesystem>
#include <sys/io.h>
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
    void VerticesRectify(cv::RotatedRect &input, cv::Point2f *output, RectType type);

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

    /// Solve the estimate launch angle by parameters given.
    /// \param v0 Initial speed of bullet (m/s).
    /// \param g Acceleration of gravity (m/s^2).
    /// \param d Horizontal distance (m).
    /// \param h Height difference (m).
    /// \return Estimated launch angle (radians).
    double ProjectileAngle(double v0, double g, double d, double h);

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

    /// Solve the rotation and the translation vectors that transform a 3D point expressed in the object coordinate
    /// frame to the camera coordinate frame, using cv::SOLVEPNP_IPPE_SQUARE.
    /// \param imagePoints Destine points on image (exactly 4).
    /// \param cameraMatrix Camera matrix.
    /// \param distortionFactor Distortion factor.
    /// \param exactSize Exact size of the coordinate object (cm).
    /// \param translationVector Output translation vector.
    /// \param rotationVector Output rotation vector.
    void
    SolvePNP(cv::Point2f imagePoints[4], cv::Mat &cameraMatrix, cv::Mat &distortionFactor, const cv::Size2f &exactSize,
             cv::Mat &translationVector, cv::Mat &rotationVector, const cv::Rect &ROI = {0, 0, 0, 0});

    /// Solve height difference between barrel and target using the translation vector of target and the motor angle of
    /// gimbal.
    /// \param translationVector The translation vector of target.
    /// \param motorAngle The motor angle of gimbal, positive upwards (radians).
    /// \param offset Offset between camera and barrel (cm).
    /// \return Height difference between barrel and target (cm).
    double SolveDeltaHeight(cv::Mat &translationVector, double motorAngle, const cv::Point2f &offset = {0, 0},
                            double angleOffset = 0);

    double SolveDistance(cv::Mat &translationVector);

    void AxisRotateZ(double x, double y, double thetaZ, double &outX, double &outY);

    void AxisRotateY(double x, double z, double thetaY, double &outX, double &outZ);

    void AxisRotateX(double y, double z, double thetaX, double &outY, double &outZ);
}

#endif //RM_STANDARD2022_UTILS_H
