//
// Created by yaione on 2/26/2022.
//

#include "core.h"

namespace rm
{
    lightblob::lightblob(cv::RotatedRect box, const camp target) :
        angle(
            box.angle > 90
                ? box.angle - 90
                : box.angle + 90), target(target),
        center(box.center)
    {
        utils::reorder_vertices(box, this->vertices, RECT_TALL);

        this->size = {std::min(box.size.height, box.size.width), std::max(box.size.height, box.size.width)};
    }

    armour::armour(std::vector<lightblob> lightblobs) : observer(6, 6, 0, CV_64F)
    {
        if (lightblobs.size() != 2) return;

        // sort light blobs left to right
        std::sort(lightblobs.begin(), lightblobs.end(),
                  [](const lightblob& lightBar1, const lightblob& lightBar2)
                  {
                      return lightBar1.center.x < lightBar2.center.x;
                  });

        int i = 0, j = 3;
        for (const auto& lightBar : lightblobs)
        {
            vertices[i++] = lightBar.vertices[j--];
            vertices[i++] = lightBar.vertices[j--];
        }

        float distanceL = utils::PointDistance(vertices[0], vertices[1]);
        float distanceR = utils::PointDistance(vertices[2], vertices[3]);
        float offsetL = round((distanceL / 0.50f - distanceL) / 2);
        float offsetR = round((distanceR / 0.50f - distanceR) / 2);
        utils::ExtendCord(vertices[0], vertices[1], offsetL, icon[0], icon[1]);
        utils::ExtendCord(vertices[3], vertices[2], offsetR, icon[3], icon[2]);

        bounding_box = boundingRect(std::vector(icon, icon + 4));

        utils::CalcPerspective(vertices, vertices);
    }

    void armour::reset(const double process_noise, const double measurement_noise, const double error)
    {
        setIdentity(observer.measurementMatrix);
        setIdentity(observer.processNoiseCov, cv::Scalar::all(process_noise));
        setIdentity(observer.measurementNoiseCov, cv::Scalar::all(measurement_noise));
        setIdentity(observer.errorCovPost, cv::Scalar::all(error));

        measurement = cv::Mat::zeros(6, 1, CV_64F);

        observer.transitionMatrix = (cv::Mat_<double>(6, 6, CV_64F) <<
            1, 0, 0, 1, 0, 0,
            0, 1, 0, 0, 1, 0,
            0, 0, 1, 0, 0, 1,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1);

        initialized = false;
    }

    void armour::update(const armour& new_observation)
    {
        for (auto [fst, snd] : new_observation.identity)
        {
            identity[fst] += snd;
        }

        if (initialized)
        {
            const auto delta_tick = new_observation.timestamp - timestamp;
            const double dt = static_cast<double>(delta_tick) / cv::getTickFrequency();

            observer.transitionMatrix.at<double>(0, 3) = dt;
            observer.transitionMatrix.at<double>(1, 4) = dt;
            observer.transitionMatrix.at<double>(2, 5) = dt;

            observer.predict();

            measurement.at<double>(3) = (new_observation.position.x - measurement.at<double>(0)) / dt;
            measurement.at<double>(4) = (new_observation.position.y - measurement.at<double>(1)) / dt;
            measurement.at<double>(5) = (new_observation.position.z - measurement.at<double>(2)) / dt;

            measurement.at<double>(0) = new_observation.position.x;
            measurement.at<double>(1) = new_observation.position.y;
            measurement.at<double>(2) = new_observation.position.z;

            observer.correct(measurement);
        }
        else
        {
            measurement.at<double>(0) = new_observation.position.x;
            measurement.at<double>(1) = new_observation.position.y;
            measurement.at<double>(2) = new_observation.position.z;

            observer.correct(measurement);
            initialized = true;
        }

        timestamp = new_observation.timestamp;
    }

    void armour::update(const int64 new_timestamp)
    {
        if (!initialized) return;

        const auto delta_tick = new_timestamp - timestamp;
        const double dt = static_cast<double>(delta_tick) / cv::getTickFrequency();

        observer.transitionMatrix.at<double>(0, 3) = dt;
        observer.transitionMatrix.at<double>(1, 4) = dt;
        observer.transitionMatrix.at<double>(2, 5) = dt;

        observer.predict();
    }

    std::tuple<int, double> armour::identity_max() const
    {
        double sum = 0;
        for (const auto& [fst, snd] : identity) sum += exp(snd);

        double max = 0;
        int max_id = -1;
        for (const auto& [fst, snd] : identity)
        {
            if (const double prob = exp(snd) / sum;
                prob > max)
            {
                max = prob;
                max_id = fst;
            }
        }

        return {max_id, max};
    }

    std::tuple<int, float> armour::max_IoU(std::vector<armour> armours) const
    {
        int index = -1;
        float max = 0;
        for (int i = 0; i < armours.size(); i++)
        {
            auto intersection = bounding_box & armours[i].bounding_box;
            const auto union_area = bounding_box.area() + armours[i].bounding_box.area() - intersection.area();
            if (const float iou = intersection.area() / union_area;
                iou > max)
            {
                max = iou;
                index = i;
            }
        }
        return {index, max};
    }
}

namespace rm::utils
{
    std::vector<std::filesystem::path> list_directory_recursive(const std::filesystem::path& directory,
                                                                const std::vector<std::string>& extension_whitelist)
    {
        std::vector<std::filesystem::path> files;
        for (const auto& p : std::filesystem::recursive_directory_iterator(directory))
        {
            if (std::filesystem::is_directory(p)) continue;
            if (!extension_whitelist.empty())
            {
                auto extension = p.path().extension().string();
                auto head = extension_whitelist.begin(),
                     tail = extension_whitelist.end();
                if (auto iterator = find(head, tail, extension); iterator != tail)
                    files.push_back(p.path());
            }
            else files.push_back(p.path());
        }
        return files;
    }

    std::vector<cv::Mat> read_image_recursive(const std::filesystem::path& directory)
    {
        std::vector<cv::Mat> images;
        const auto files = list_directory_recursive(directory, {".jpg"});
        for_each(files.begin(), files.end(), [&images](const std::filesystem::path& p)
        {
            cv::Mat image = cv::imread(p.string());
            resize(image, image, {20, 20}, 0, 0, cv::INTER_LINEAR);
            image = image.reshape(1, 1);
            image.convertTo(image, CV_32FC1);
            images.push_back(image);
        });
        return images;
    }

    cv::Mat flatten_image(const cv::Mat& input, const int data_type, const cv::Size image_size)
    {
        cv::Mat image;
        input.copyTo(image);

        if (image_size.width != 0 && image_size.height != 0)
        {
            resize(image, image, image_size, 0, 0, cv::INTER_LINEAR);
        }

        image = image.reshape(1, 1);
        image.convertTo(image, data_type);

        return image;
    }

    cv::Rect GetROI(cv::Point2f* imagePoints, int pointsCount, float scaleFactor, const cv::Size& frameSize,
                    const cv::Rect& previous)
    {
        return GetROI(imagePoints, pointsCount, {scaleFactor, scaleFactor}, frameSize, previous);
    }

    cv::Rect GetROI(cv::Point2f* imagePoints, int pointsCount, const cv::Size2f& scaleFactor, const cv::Size& frameSize,
                    const cv::Rect& previous)
    {
        cv::Rect boundingRect = cv::boundingRect(std::vector<cv::Point2f>(imagePoints, imagePoints + pointsCount));
        boundingRect.x += previous.x;
        boundingRect.y += previous.y;
        if (scaleFactor.width != 1.0f || scaleFactor.height != 1.0f)
        {
            cv::Size scale((int)((double)boundingRect.width * scaleFactor.width / 2.0),
                           (int)((double)boundingRect.height * scaleFactor.height / 2.0));

            boundingRect.x -= scale.width;
            boundingRect.y -= scale.height;
            boundingRect.width += scale.width * 2;
            boundingRect.height += scale.width * 2;
        }
        if (boundingRect.x < 0)
        {
            boundingRect.x = 0;
        }
        if (boundingRect.y < 0)
        {
            boundingRect.y = 0;
        }
        if (boundingRect.x + boundingRect.width >= frameSize.width)
        {
            boundingRect.width = frameSize.width - boundingRect.x - 1;
        }
        if (boundingRect.y + boundingRect.height >= frameSize.height)
        {
            boundingRect.height = frameSize.height - boundingRect.y - 1;
        }

        if (boundingRect.width < 0 || boundingRect.height < 0)
        {
            return {0, 0, 0, 0};
        }

        return boundingRect;
    }

    void reorder_vertices(cv::RotatedRect& input, cv::Point2f* output, RectType type = RECT_TALL)
    {
        cv::Point2f temp[4];
        input.points(temp);

        // sort vertices ascending by y
        std::sort(temp, temp + 4, [](const cv::Point2f& point1, const cv::Point2f& point2)
        {
            return point1.y < point2.y;
        });

        bool swap_up = temp[0].x < temp[1].x, swap_down = temp[2].x < temp[3].x;
        output[0] = swap_down ? temp[2] : temp[3]; // left down
        output[1] = swap_up ? temp[0] : temp[1]; // left up
        output[2] = swap_up ? temp[1] : temp[0]; // right up
        output[3] = swap_down ? temp[3] : temp[2]; // right down

        // TODO: Add side support & handle some edge case.
    }

    float PointDistance(const cv::Point2f& pt1, const cv::Point2f& pt2)
    {
        return (float)sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
    }

    float PointDistance(const cv::Point2i& pt1, const cv::Point2i& pt2)
    {
        return (float)sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
    }

    void
    ExtendCord(const cv::Point2f& pt1, const cv::Point2f& pt2, float deltaLen, cv::Point2f& dst1, cv::Point2f& dst2)
    {
        // Special case
        if (pt1.x == pt2.x)
        {
            dst1.x = pt1.x;
            dst2.x = pt1.x;

            if (pt1.y > pt2.y)
            {
                dst1.y = pt1.y + deltaLen;
                dst2.y = pt2.y - deltaLen;
            }
            else
            {
                dst1.y = pt1.y - deltaLen;
                dst2.y = pt2.y + deltaLen;
            }
        }
        else if (pt1.y == pt2.y)
        {
            dst1.y = pt1.y;
            dst2.y = pt1.y;

            if (pt1.x > pt2.x)
            {
                dst1.x = pt1.x + deltaLen;
                dst2.x = pt2.x - deltaLen;
            }
            else
            {
                dst1.x = pt1.x - deltaLen;
                dst2.x = pt2.x + deltaLen;
            }
            // Common case
        }
        else
        {
            float k = (float)(pt1.y - pt2.y) / (float)(pt1.x - pt2.x);
            float theta = atan2((float)abs(pt1.y - pt2.y), (float)abs(pt1.x - pt2.x));
            float zoomY = sin(theta) * deltaLen;
            float zoomX = cos(theta) * deltaLen;

            // Left tilt
            if (k > 0)
            {
                if (pt1.x > pt2.x)
                {
                    dst1.x = pt1.x + zoomX;
                    dst1.y = pt1.y + zoomY;

                    dst2.x = pt2.x - zoomX;
                    dst2.y = pt2.y - zoomY;
                }
                else
                {
                    dst1.x = pt1.x - zoomX;
                    dst1.y = pt1.y - zoomY;

                    dst2.x = pt2.x + zoomX;
                    dst2.y = pt2.y + zoomY;
                }
                // Right tilt
            }
            else
            {
                if (pt1.x < pt2.x)
                {
                    dst1.x = pt1.x - zoomX;
                    dst1.y = pt1.y + zoomY;

                    dst2.x = pt2.x + zoomX;
                    dst2.y = pt2.y - zoomY;
                }
                else
                {
                    dst1.x = pt1.x + zoomX;
                    dst1.y = pt1.y - zoomY;

                    dst2.x = pt2.x - zoomX;
                    dst2.y = pt2.y + zoomY;
                }
            }
        }
    }

    void CalcPerspective(cv::Point2f input[4], cv::Point2f output[4], const float outRatio)
    {
        const float leftHeight = PointDistance(input[0], input[1]);
        const float rightHeight = PointDistance(input[2], input[3]);
        const float maxHeight = fmax(leftHeight, rightHeight);

        const cv::Size2f size(maxHeight * outRatio, maxHeight);
        const cv::Point2f center = LineCenter(LineCenter(input[0], input[1]), LineCenter(input[2], input[3]));

        output[0].x = center.x - size.width / 2;
        output[0].y = center.y - size.height / 2;
        output[1].x = center.x - size.width / 2;
        output[1].y = center.y + size.height / 2;
        output[2].x = center.x + size.width / 2;
        output[2].y = center.y + size.height / 2;
        output[3].x = center.x + size.width / 2;
        output[3].y = center.y - size.height / 2;
    }

    cv::Point2f LineCenter(const cv::Point2f& pt1, const cv::Point2f& pt2)
    {
        return {pt1.x / 2 + pt2.x / 2, pt1.y / 2 + pt2.y / 2};
    }

    cv::Mat euler2homogeneous(const euler<double>& rotation)
    {
        return euler2homogeneous(rotation.x, rotation.y, rotation.z);
    }

    cv::Mat euler2homogeneous(const double x, const double y, const double z)
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

        const cv::Mat matrix = r_z * r_y * r_x;
        const cv::Mat homogeneous = cv::Mat::eye(4, 4, CV_64F);
        matrix.copyTo(homogeneous(cv::Rect(0, 0, 3, 3)));

        return homogeneous;
    }
}
