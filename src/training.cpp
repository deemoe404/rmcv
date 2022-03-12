//
// Created by yaione on 3/12/22.
//

#include "rmcv/mlp/training.h"

namespace rm::mlp {
    std::string i2s(int number) {
        char str[64] = {0};
        std::snprintf(str, 64, "%d", number);
        std::string res(str);
        return res;
    }

    // TODO: Path combine
    void Labeling(const std::string &folder, int fileCounts) {
        cv::Mat image;
        while (fileCounts--) {
            std::string filename = i2s(fileCounts) + ".jpg";
            image = cv::imread(folder + filename);
            cv::imshow("preview", image);
            std::string key = i2s(cv::waitKey(0) - 48);
            std::cout << key << std::endl;
            cv::imwrite(folder + key + "/" + filename, image);
        }
    }

}
