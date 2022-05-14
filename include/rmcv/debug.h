//
// Created by yaione on 3/17/22.
//

#ifndef RM_STANDARD2022_DEBUG_H
#define RM_STANDARD2022_DEBUG_H

#include "core/core.h"

namespace rm::debug {
    void DrawArmours(const std::vector<rm::Armour> &input, cv::Mat &output, int index);

    void DrawArmour(rm::Armour input, cv::Mat &output);

    void DrawlightBlobs(const std::vector<rm::LightBlob> &input, cv::Mat &output, int index);
}

#endif //RM_STANDARD2022_DEBUG_H
