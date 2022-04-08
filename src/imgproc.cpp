//
// Created by yaione on 2/26/2022.
//

#include "rmcv/imgproc.h"

namespace rm {
    void CalcRatio(cv::Mat &source, cv::Mat &calibration, cv::Point vertices[4], cv::Size outSize) {
        // handel the case of vertices out of screen
        for (int i = 0; i < 4; i++) {
            if (vertices[i].x < 0 || vertices[i].y < 0 || vertices[i].x > source.cols - 1 ||
                vertices[i].y > source.rows - 1) {
                if (vertices[i].x < 0) vertices[i].x = 0;
                if (vertices[i].y < 0) vertices[i].y = 0;
                if (vertices[i].x > source.cols - 1) vertices[i].x = source.cols - 1;
                if (vertices[i].y > source.rows - 1) vertices[i].y = source.rows - 1;
            }
        }

        cv::Rect box = cv::boundingRect(std::vector<cv::Point>({vertices[0], vertices[1], vertices[2], vertices[3]}));
        cv::Point2f srcPts[3] = {{(float) (vertices[1].x - box.x), (float) (vertices[1].y - box.y)},
                                 {(float) (vertices[2].x - box.x), (float) (vertices[2].y - box.y)},
                                 {(float) (vertices[0].x - box.x), (float) (vertices[0].y - box.y)}};
        cv::Point2f dstPts[3] = {{0,                   0},
                                 {(float) (box.width), 0},
                                 {0,                   (float) (box.height)}};
        auto warp = cv::getAffineTransform(srcPts, dstPts);
        cv::warpAffine(source(box), calibration, warp, calibration.size());

        // resize to the dst size
        cv::resize(calibration, calibration, outSize);
    }

    void CalcGamma(cv::Mat &source, cv::Mat &calibration, float gamma) {
        cv::Mat lookUpTable(1, 256, CV_8U);
        uchar *p = lookUpTable.ptr();
        for (int i = 0; i < 256; ++i) {
            p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
        }

        // search for LUT table
        cv::LUT(source, lookUpTable, calibration);
    }

    void
    ExtractColor(cv::InputArray image, cv::OutputArray binary, rm::CampType ownCamp, bool overExposed, int lowerBound,
                 cv::Size kernelSize) {
        if (overExposed) {
            cv::inRange(image, ownCamp == rm::CAMP_RED ? cv::Scalar{250, (double) lowerBound, (double) lowerBound}
                                                       : cv::Scalar{(double) lowerBound, (double) lowerBound, 250},
                        cv::Scalar{255, 255, 255}, binary);
        } else {
            std::vector<cv::Mat> channels;
            cv::split(image, channels);

            // extract color
            auto gray = channels[ownCamp == rm::CAMP_RED ? 0 : 2] - channels[ownCamp == rm::CAMP_RED ? 2 : 0];
            cv::inRange(gray, lowerBound, 255, binary);
        }

        // enhance visual
        if (kernelSize.width != 0 && kernelSize.height != 0) {
            auto kernel = cv::getStructuringElement(cv::MORPH_RECT, {kernelSize.width, kernelSize.height});
            cv::morphologyEx(binary, binary, cv::MORPH_DILATE, kernel);
        }
    }

    void AutoEnhance(cv::Mat &frame, float maxGainFactor, float minGainFactor) {
        cv::Scalar meanValue = cv::mean(frame);
        float meanC3 = (float) (meanValue[0] + meanValue[1] + meanValue[2]) / 3.0f;
        float k = 2.0f / (maxGainFactor - minGainFactor);
        float b = 3.0f - maxGainFactor * k;
        float gammaFactor = k * meanC3 + b;

        // Map [1.0, -3.0] to [1.0, 0.0]
        if (gammaFactor <= 1.0f && gammaFactor >= -3.0f) {
            gammaFactor = 1.0f + (gammaFactor - 1) / 4.0f;
        } else if (gammaFactor < -3.0f) {
            // Frame too dark, gamma might not helpful in this case.
            gammaFactor = 0;
        }

        rm::CalcGamma(frame, frame, gammaFactor);
    }

    void AutoBinarize(cv::Mat &image, cv::Mat &binary) {
        cv::cvtColor(image, binary, cv::COLOR_BGR2GRAY);
        cv::Scalar meanValue = cv::mean(binary);
        cv::inRange(binary, cv::Scalar{meanValue[0], meanValue[1], meanValue[2]}, cv::Scalar{255, 255, 255}, binary);

        binary.convertTo(binary, CV_32FC1, 1.0 / 255.0);
    }
}
