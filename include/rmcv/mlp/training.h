//
// Created by yaione on 3/12/22.
//

#ifndef RM_STANDARD2022_TRAINING_H
#define RM_STANDARD2022_TRAINING_H

#include "string"
#include "opencv2/opencv.hpp"
#include "opencv2/ml.hpp"
#include "rmcv/core/core.h"

namespace rm::mlp {
    void Labeling(const std::string &folder, int fileCounts);

    void TrainMLP(const std::string &path, const std::vector<int> &labels, int imgCount, cv::Ptr<cv::ml::ANN_MLP> &model);
}

#endif //RM_STANDARD2022_TRAINING_H
