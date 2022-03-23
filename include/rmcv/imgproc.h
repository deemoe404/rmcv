//
// Created by yaione on 2/26/2022.
//

#ifndef RM_STANDARD2022_IMGPROC_H
#define RM_STANDARD2022_IMGPROC_H

#include "rmcv/core/core.h"

namespace rm {
    void CalcRatio(cv::Mat &input, cv::Mat &output, cv::Point vertices[4], cv::Rect box, cv::Size outSize);

    void CalcGamma(cv::Mat &input, cv::Mat &output, float gamma = 0.5f);

    void ExtractColor(cv::Mat &input, cv::Mat &output, rm::CampType camp);

    void EnhanceIcon();
}

#endif //RM_STANDARD2022_IMGPROC_H
