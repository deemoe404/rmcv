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

    void CalcGamma(const cv::Mat &input, cv::Mat &output, const float gamma = 0.5f) {
        unsigned char bin[256];
        for (int i = 0; i < 256; ++i) {
            bin[i] = cv::saturate_cast<uchar>(pow((float) (i / 255.0), gamma) * 255.0f);
        }
        output = input.clone();
        const int channels = output.channels();
        switch (channels) {
            case 1: {
                cv::MatIterator_<uchar> it, end;
                for (it = output.begin<uchar>(), end = output.end<uchar>(); it != end; it++)
                    *it = bin[(*it)];
                break;
            }
            case 3: {
                cv::MatIterator_<cv::Vec3b> it, end;
                for (it = output.begin<cv::Vec3b>(), end = output.end<cv::Vec3b>(); it != end; it++) {
                    (*it)[0] = bin[((*it)[0])];
                    (*it)[1] = bin[((*it)[1])];
                    (*it)[2] = bin[((*it)[2])];
                }
                break;
            }
        }
    }

    void ExtractColor(const cv::Mat &input, cv::Mat &output, const rm::CampType enemy) {
        cv::Mat hsv;
        cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);

        if (enemy == rm::CAMP_BLUE) {
            // TODO: test blue extraction
        } else if (enemy == rm::CAMP_RED) {
            // TODO: test red extraction
        }

        cv::Mat bin;
        cv::adaptiveThreshold(bin, bin, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 25, 5);
    }
}
