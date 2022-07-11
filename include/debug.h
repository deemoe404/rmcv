//
// Created by yaione on 3/17/22.
//

#ifndef RMCV_DEBUG_H
#define RMCV_DEBUG_H

#include "core/core.h"

/// \brief Debug utilities.
namespace rm::debug {
    void DrawArmours(const std::vector<rm::Armour> &input, cv::Mat &output, int index);

    void DrawArmour(rm::Armour input, cv::Mat &output);

    void DrawLightBlobs(const std::vector<rm::LightBlob> &input, cv::Mat &output, int index);

    void PrintMat(cv::Mat &input, int decimal = 0);
}

#endif //RMCV_DEBUG_H
