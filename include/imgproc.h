//
// Created by yaione on 2/26/2022.
//

#ifndef RMCV_IMGPROC_H
#define RMCV_IMGPROC_H

#include "core.h"

namespace rm
{
    /// Calibrate a portion of the source frame to the destine rect using Affine Transformation Correction.
    /// \param source Source image.
    /// \param vertices Vertices of the portion on the source frame.
    /// \param outSize Destine size.
    /// \return Calibrated image.
    cv::Mat affine_correction(const cv::Mat& source, cv::Point2f vertices[4], cv::Size outSize);

    /// Perform a gamma transform on the input image.
    /// \param source Source image.
    /// \param calibration Output calibrated image.
    /// \param gamma Gamma factor.
    void CalcGamma(cv::Mat& source, cv::Mat& calibration, float gamma = 0.5f);

    /// Extract specified color from source image.
    /// \param image Source image.
    /// \param target Specify the camp of the color to be extracted.
    /// \param lower_bound Lower bound when performing binarization.
    std::tuple<std::vector<contour>, cv::Mat> extract_color(cv::InputArray image, camp target, int lower_bound);

    /// Auto enhance image by the given benchmarks.
    /// \param frame Source image & destine image.
    /// \param maxGainFactor The mean value of pixel values where gain should be maximize.
    /// \param minGainFactor The mean value of pixel values where gain should be minimize.
    void AutoEnhance(cv::Mat& frame, float maxGainFactor = 100.0, float minGainFactor = 50.0);

    /// Use the mean value of each channels to binarize given image and normalize to CV_32FC1.
    /// \param image Source image.
    /// \param binary Destine image.
    void AutoBinarize(cv::Mat& image, cv::Mat& binary);
}

#endif //RMCV_IMGPROC_H
