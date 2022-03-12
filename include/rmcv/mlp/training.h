//
// Created by yaione on 3/12/22.
//

#ifndef RM_STANDARD2022_TRAINING_H
#define RM_STANDARD2022_TRAINING_H

#include "string"
#include "opencv2/opencv.hpp"
#include "opencv2/ml.hpp"

namespace rm::mlp {
    std::string i2s(int number);

    void Labeling(const std::string &folder, int fileCounts);
}

#endif //RM_STANDARD2022_TRAINING_H
