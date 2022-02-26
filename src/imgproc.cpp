//
// Created by yaione on 2/26/2022.
//

#include "imgproc.h"

namespace rm {
    void CalcRatio(const cv::Point2f srcPts[4], const cv::Point2f dstPts[4], const cv::Mat &input, cv::Mat &output) {
        cv::Mat tmp;
        auto warp = cv::getAffineTransform(srcPts, dstPts);
        cv::warpAffine(input, tmp, warp, tmp.size());
        cv::Rect roi(0, 0, dstPts[1].x, dstPts[1].x);
        tmp(roi).copyTo(output);
    }

    void ExtractColor(const cv::Mat &input, cv::Mat &output, const rm::CampType enemy) {
        cv::Mat hsv;
        cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);
        if (enemy == rm::CAMP_BLUE) {
            // TODO: test blue extraction
        } else if (enemy == rm::CAMP_RED) {
            // TODO: test red extraction
        }
    }
}
