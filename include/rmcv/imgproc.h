//
// Created by yaione on 2/26/2022.
//

#ifndef RM_STANDARD2022_IMGPROC_H
#define RM_STANDARD2022_IMGPROC_H

#include "rmcv/core/core.h"

namespace rm {
    /// Calibrate a portion of the source frame to the destine rect.
    /// \param source Source image.
    /// \param calibration Output calibrated image.
    /// \param vertices Vertices of the portion on the source frame.
    /// \param outSize Destine size.
    void CalcRatio(cv::Mat &source, cv::Mat &calibration, cv::Point vertices[4], cv::Size outSize);

    /// Perform a gamma transform on the input image.
    /// \param source Source image.
    /// \param calibration Output calibrated image.
    /// \param gamma Gamma factor.
    void CalcGamma(cv::Mat &source, cv::Mat &calibration, float gamma = 0.5f);

    /// Extract enemy color from source image.
    /// \param image Source image.
    /// \param binary Output binary image.
    /// \param ownCamp Own camp.
    /// \param overExposed Specify if the image has been over exposed. If true, the color of own camp would be
    ///                    extracted somehow, consider pass source image to rm::FindLightBars function when finding
    ///                    light bars.
    /// \param lowerBound Lower bound when performing binarization.
    /// \param kernelSize Kernel size when performing dilate.
    void
    ExtractColor(cv::InputArray image, cv::OutputArray binary, rm::CampType ownCamp, bool overExposed, int lowerBound,
                 cv::Size kernelSize);

    /// Auto enhance image by the given benchmarks.
    /// \param frame Source image & destine image.
    /// \param maxGainFactor The mean value of pixel values where gain should be maximize.
    /// \param minGainFactor The mean value of pixel values where gain should be minimize.
    void AutoEnhance(cv::Mat &frame, float maxGainFactor = 100.0, float minGainFactor = 50.0);

    /// Use the mean value of each channels to binarize given image and normalize to CV_32FC1.
    /// \param image Source image.
    /// \param binary Destine image.
    void AutoBinarize(cv::Mat &image, cv::Mat &binary);
}

#endif //RM_STANDARD2022_IMGPROC_H
