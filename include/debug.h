//
// Created by yaione on 3/17/22.
//

#ifndef RMCV_DEBUG_H
#define RMCV_DEBUG_H

#include "core.h"

/// \brief Debug utilities.
namespace rm::debug
{
    void draw_armours(const std::vector<armour>& input, cv::Mat& output, int index);

    void DrawArmour(armour input, cv::Mat& output);

    void draw_lightblobs(const std::vector<lightblob>& positive, const std::vector<contour>& negative, cv::Mat& output,
                        int index);

    void PrintMat(cv::Mat& input, int decimal = 0);
}

#endif //RMCV_DEBUG_H
