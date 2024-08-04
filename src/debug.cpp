//
// Created by yaione on 3/17/22.
//

#include "debug.h"

namespace rm::debug
{
    logger::logger(const int64 section_id, int FPS, const cv::Size& resolution): section_id(section_id)
    {
        const auto path = std::filesystem::current_path() / std::to_string(section_id);
        if (!exists(path))
        {
            create_directory(path);
            stream_writer.open(
                (path / "video.avi").string(), cv::VideoWriter::fourcc('F', 'F', 'V', '1'), FPS, resolution);
            reading = false;
        }
        else
        {
            stream_reader.open((path / "video.avi").string());
            reading = true;
        }
        storage.open((path / "metadata.xml").string(), reading ? cv::FileStorage::READ : cv::FileStorage::WRITE);
    }

    logger::~logger()
    {
        storage.release();
        if (reading) stream_reader.release();
        else stream_writer.release();
    }

    void logger::write(const cv::Mat& image, const cv::Mat& data)
    {
        if (reading) return;

        stream_writer.write(image);
        storage << "frame" << frame_id++;
        storage << "data" << data;
    }

    void draw_armours(const std::vector<armour>& input, cv::Mat& output, const int index)
    {
        if (input.empty()) return;

        std::vector<contour> contours;
        auto process_armour = [&](const armour& ar)
        {
            contours.emplace_back(ar.vertices, ar.vertices + 4);
            contours.emplace_back(ar.icon, ar.icon + 4);

            const std::string position = std::to_string(ar.identity) + ": " +
                std::to_string(ar.position.x) + ", " +
                std::to_string(ar.position.y) + ", " +
                std::to_string(ar.position.z);
            putText(output, position, ar.vertices[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 255});
        };

        if (index < 0 || index >= input.size())
        {
            for (const auto& ar : input) process_armour(ar);
        }
        else
        {
            process_armour(input[index]);
        }

        drawContours(output, contours, -1, {0, 255, 255}, 1);
    }

    void draw_lightblobs(const std::vector<lightblob>& positive, const std::vector<contour>& negative, cv::Mat& output,
                         const int index)
    {
        if (positive.empty() && negative.empty()) return;

        auto draw_lightblob = [&](const lightblob& lightblob)
        {
            const std::vector<cv::Point> vertices(std::begin(lightblob.vertices), std::end(lightblob.vertices));
            const std::vector<std::vector<cv::Point>> contours = {vertices};

            const cv::Scalar color = lightblob.target == CAMP_RED ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
            drawContours(output, contours, -1, color, 1);
        };

        if (index < 0 || index > positive.size())
            for (auto& lightblob : positive) draw_lightblob(lightblob);
        else
            draw_lightblob(positive[index]);

        if (!negative.empty())
            drawContours(output, negative, -1, {0, 255, 255}, 1);
    }
}
