//
// Created by yaione on 3/12/22.
//

#ifndef RM_STANDARD2022_TRAINING_H
#define RM_STANDARD2022_TRAINING_H

#include "string"
#include "sys/stat.h"
#include "sys/types.h"
#include "opencv2/opencv.hpp"
#include "opencv2/ml.hpp"
#include "rmcv/core/core.h"

namespace rm::mlp {
    enum CaptureType {
        CAPTURE_MIXED = -1, CAPTURE_LABEL = 0
    };

    /// Capture the image in the structure of rm::TrainMLP needed.
    /// \param path Work folder. Sub-folders will be created automatically if CAPTURE_LABEL is selected.
    /// \param input Image.
    /// \param type Specify capture type.
    /// \param label Label of this image. Ignore if CAPTURE_MIXED is selected.
    void CaptureImage(const std::string &path, cv::Mat &input, rm::mlp::CaptureType type = rm::mlp::CAPTURE_MIXED,
                      int label = 0);
    // TODO: Rename all pictures.

    /// Labeling all images under work folder.
    /// \param path Work folder.
    /// \param fileCounts File counts.
    void Labeling(const std::string &path, int fileCounts);
    // TODO: Auto mkdir.

    /// Train a MLP model using default parameters and given info. Image files must be named starting from 0 and in the format of jpg.
    /// \param path The path for all images. Must containing sub-folders for each labels.
    /// \param labels Labels. Output of MLP will be same as the order in this input.
    /// \param imgCount Image count for every labels.
    /// \param model Output/
    void
    TrainMLP(const std::string &path, const std::vector<int> &labels, int imgCount, cv::Ptr<cv::ml::ANN_MLP> &model);
}

#endif //RM_STANDARD2022_TRAINING_H
