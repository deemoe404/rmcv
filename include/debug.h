//
// Created by yaione on 3/17/22.
//

#ifndef RMCV_DEBUG_H
#define RMCV_DEBUG_H

#include "core.h"

/// \brief Debug utilities.
namespace rm::debug
{
    class logger
    {
        cv::FileStorage storage;
        cv::VideoCapture stream_reader;
        cv::VideoWriter stream_writer;

        int64 section_id = 0;
        int64 frame_id = 0;
        bool reading = true;

    public:
        explicit logger(int64 section_id = -1, int FPS = 210, const cv::Size& resolution = {1280, 1024});

        ~logger();

        void write(const cv::Mat& image, const cv::Mat& data);
    };

    void draw_armours(const std::vector<armour>& input, cv::Mat& output, int index);

    void DrawArmour(armour input, cv::Mat& output);

    void draw_lightblobs(const std::vector<lightblob>& positive, const std::vector<contour>& negative, cv::Mat& output,
                        int index);

    void PrintMat(cv::Mat& input, int decimal = 0);
}

#endif //RMCV_DEBUG_H
