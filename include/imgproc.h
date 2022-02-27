//
// Created by yaione on 2/26/2022.
//

#ifndef RM_STANDARD2022_IMGPROC_H
#define RM_STANDARD2022_IMGPROC_H

#include "core.h"

namespace rm {
    void CalcRatio(const cv::Point2f srcPts[4], const cv::Point2f dstPts[4], const cv::Mat &input, cv::Mat &output);

    void CalcGamma(const cv::Mat &input, cv::Mat &output, const float gamma);

    void ExtractColor(const cv::Mat &input, cv::Mat &output, const rm::CampType enemy);
}

#endif //RM_STANDARD2022_IMGPROC_H
