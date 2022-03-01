//
// Created by yaione on 2/26/2022.
//

#ifndef RM_STANDARD2022_IMGPROC_H
#define RM_STANDARD2022_IMGPROC_H

#include "core.h"

namespace rm {
    void CalcRatio(cv::Mat &input, cv::Mat &output, cv::Point2f srcPts[4], cv::Point2f dstPts[4]);

    void CalcGamma(cv::Mat &input, cv::Mat &output, float gamma);

    void ExtractColor(cv::Mat &input, cv::Mat &output, rm::CampType enemy);
}

#endif //RM_STANDARD2022_IMGPROC_H
