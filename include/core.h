//
// Created by yaione on 2/26/2022.
//

#ifndef RMCV_CORE_H
#define RMCV_CORE_H

#include <cmath>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <dirent.h>
#include <filesystem>
#include <cstring>
#include "opencv2/opencv.hpp"


#include <string>
#include <thread>
#include <algorithm>
#include <filesystem>
#include <random>
#include <opencv2/ml.hpp>

namespace rm
{
    enum camp
    {
        CAMP_RED = 0, CAMP_BLUE = 1, CAMP_GUIDELIGHT = 2, CAMP_NEUTRAL = -1
    };

    enum AimMode
    {
        AIM_COMBAT = 0, AIM_BUFF = 1, AIM_SNIPE = 2
    };

    enum RectType
    {
        RECT_TALL = 0, RECT_SIDE = 1
    };

    enum FileType
    {
        FILETYPE_REGULAR_FILE = 0,
        FILETYPE_DIRECTORY = 1,
        FILETYPE_SYMBOLIC_LINK = 2,
        FILETYPE_SOCKET = 3,
        FILETYPE_UNKNOWN = -1
    };

    template <typename T>
    struct range
    {
        T lower_bound;
        T upper_bound;

        range(T lower, T upper) : lower_bound(lower), upper_bound(upper)
        {
        }

        [[nodiscard]] bool contains(T value) const
        {
            return value >= lower_bound && value <= upper_bound;
        }
    };

    struct parameters
    {
        int extraction_lower_bound = 80;

        float lightblob_tilt_max = 70.0;
        range<float> lightblob_ratio = {1.5, 80.0};
        range<double> lightblob_area = {10, 99999};

        float armour_angle_difference_max = 12;
        float armour_shear_max = 22;
        float armour_lenght_ratio_max = 12;

        parameters() = default;

        explicit parameters(const std::string& path)
        {
        }
    };

    typedef std::vector<cv::Point> contour;

    class lightblob
    {
    public:
        float angle = 0; /// Rotation angle of the light blob (vertical when the angle is 90)
        camp target = CAMP_NEUTRAL; /// Camp this light blob belongs to
        cv::Point2f center; /// Mass center of the light blob
        cv::Point2f vertices[4]; /// Four vertices around the light blob
        cv::Size2f size; /// Width and height of the light blob

        lightblob() = default;

        explicit lightblob(cv::RotatedRect box, rm::camp camp = rm::CAMP_NEUTRAL);
    };

    class armour
    {
        cv::KalmanFilter observer;
        cv::Mat measurement = cv::Mat::zeros(6, 1, CV_32F);
        bool initialized = false;

    public:
        float rank = 0; /// A value help in sorting multiple armours
        camp identity = CAMP_NEUTRAL; /// Camp this armour belongs to
        cv::Point2f icon[4]; /// Vertices of icon area
        cv::Point2f vertices[4]; /// Vertices of armour (square with light blob as side length for better PNP result)

        cv::Point3d position; /// Position of the armour in 3D space
        int64 timespan = 0;
        int lost_count = 0;
        std::map<int, int> id_count = {};

        armour() = default;

        explicit armour(std::vector<lightblob> lightblobs, float rank = 0, camp target = CAMP_NEUTRAL);

        void reset(double process_noise, double measurement_noise);
    };
}

namespace rm::utils
{
    std::vector<std::filesystem::path> list_directory_recursive(const std::filesystem::path& directory,
                                                                const std::vector<std::string>& extension_whitelist =
                                                                    {});

    std::vector<cv::Mat> read_image_recursive(const std::filesystem::path& directory);

    cv::Mat flatten_image(const cv::Mat& input, int data_type, cv::Size image_size);

    FileType GetFileType(const char* filename);

    bool ListFiles(const char* path, std::vector<std::string>& filenames);

    cv::Rect
    GetROI(cv::Point2f* imagePoints, int pointsCount, float scaleFactor = 1.0f, const cv::Size& frameSize = {-1, -1},
           const cv::Rect& previous = {0, 0, 0, 0});

    cv::Rect GetROI(cv::Point2f* imagePoints, int pointsCount, const cv::Size2f& scaleFactor = {1, 1},
                    const cv::Size& frameSize = {-1, -1}, const cv::Rect& previous = {0, 0, 0, 0});

    /// Reorder vertices of a rectangle to match the point order in left down, left up, right up, right down.
    /// \param input The original rectangle.
    /// \param output Reordered vertices.
    /// \param type Rectangle type.
    void reorder_vertices(cv::RotatedRect& input, cv::Point2f* output, RectType type);

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
    NewtonIteration(double (*fd)(double, std::vector<double>), const std::vector<double>& literals, double x0 = 0,
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
    float PointDistance(const cv::Point2f& pt1, const cv::Point2f& pt2);

    /// Return the distance between two given points.
    /// \param pt1 First point.
    /// \param pt2 Second point.
    /// \return Distance.
    float PointDistance(const cv::Point2i& pt1, const cv::Point2i& pt2);

    /// Return the midpoint between two given points.
    /// \param pt1 First point.
    /// \param pt2 Second point.
    /// \return Midpoint.
    cv::Point2f LineCenter(const cv::Point2f& pt1, const cv::Point2f& pt2);

    /// Expand the cord by the given length while center of the cord remain still.
    /// \param pt1 First point of the cord.
    /// \param pt2 Second point of the cord.
    /// \param deltaLen The length to be expanded.
    /// \param dst1 The first point after expanding.
    /// \param dst2 The second point after expanding.
    void
    ExtendCord(const cv::Point2f& pt1, const cv::Point2f& pt2, float deltaLen, cv::Point2f& dst1, cv::Point2f& dst2);

    std::string PathCombine(const std::string& path1, const std::string& path2);

    std::string int2str(int number);

    /// Convert euler angles to rotation matrix.
    /// \param x Rotation angle around x axis in radians.
    /// \param y Rotation angle around y axis in radians.
    /// \param z Rotation angle around z axis in radians.
    cv::Mat euler2matrix(double x, double y, double z);
}

#endif //RMCV_CORE_H
