//
// Created by yaione on 2/26/2022.
//

#ifndef RMCV_CORE_H
#define RMCV_CORE_H

#include <cmath>
#include <vector>
#include <string>
#include <filesystem>
#include <thread>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>

namespace rm
{
    enum camp
    {
        CAMP_RED = 0, CAMP_BLUE = 1, CAMP_GUIDELIGHT = 2, CAMP_NEUTRAL = -1
    };

    enum RectType
    {
        RECT_TALL = 0, RECT_SIDE = 1
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

    template <typename T>
    struct euler
    {
        T x;
        T y;
        T z;

        euler(T x, T y, T z) : x(x), y(y), z(z)
        {
        }

        explicit euler(cv::Mat array) : x(array.at<T>(0)), y(array.at<T>(1)), z(array.at<T>(2))
        {
        }

        [[nodiscard]] cv::Mat to_array() const
        {
            return cv::Mat_<T>(3, 1) << x, y, z;
        }

        [[nodiscard]] cv::Mat to_matrix() const
        {
            const cv::Mat r_z = (cv::Mat_<double>(3, 3, CV_64F) <<
                std::cos(z), -std::sin(z), 0,
                std::sin(z), std::cos(z), 0,
                0, 0, 1);

            const cv::Mat r_y = (cv::Mat_<double>(3, 3, CV_64F) <<
                std::cos(y), 0, std::sin(y),
                0, 1, 0,
                -std::sin(y), 0, std::cos(y));

            const cv::Mat r_x = (cv::Mat_<double>(3, 3, CV_64F) <<
                1, 0, 0,
                0, std::cos(x), -std::sin(x),
                0, std::sin(x), std::cos(x));

            return r_z * r_y * r_x;
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

        explicit lightblob(cv::RotatedRect box, rm::camp camp = rm::CAMP_NEUTRAL);
    };

    class armour
    {
        std::map<int, int> identity_history = {};

        cv::KalmanFilter observer;
        cv::Mat measurement = cv::Mat::zeros(6, 1, CV_32F);
        bool initialized = false;

    public:
        cv::Point2f icon[4]; /// Vertices of icon area
        cv::Point2f vertices[4]; /// Vertices of armour (square with light blob as side length for better PNP result)
        cv::Rect2f bounding_box; /// Bounding box of the armour

        int64 timestamp = 0;
        int lost_count = 0;
        cv::Point3d position;
        int identity = -1;

        explicit armour(std::vector<lightblob> lightblobs);

        void reset(double process_noise, double measurement_noise, double error);

        void update(const armour& new_observation);

        void update(int64 new_timestamp);

        [[nodiscard]] std::tuple<int, double> identity_max() const;

        [[nodiscard]] std::tuple<int, float> max_IoU(std::vector<armour> armours) const;
    };
}

namespace rm::utils
{
    std::vector<std::filesystem::path> list_directory_recursive(
        const std::filesystem::path& directory, const std::vector<std::string>& extension_whitelist = {});

    std::vector<cv::Mat> read_image_recursive(const std::filesystem::path& directory);

    cv::Mat flatten_image(const cv::Mat& input, int data_type, cv::Size image_size = {0, 0});

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

    cv::Mat homogeneous(const cv::Mat& rotation, const cv::Mat& translation = cv::Mat::zeros(3, 1, CV_64F));
}

#endif //RMCV_CORE_H
