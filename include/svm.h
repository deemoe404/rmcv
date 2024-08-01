//
// Created by Sam Cheung on 7/23/24.
//

#ifndef SVM_H
#define SVM_H

#include <core.h>
#include <random>

namespace rm::svm
{
    class dataset : public std::map<int, std::vector<cv::Mat>>
    {
    public:
        std::vector<std::string> labels;

        explicit dataset(const std::vector<std::string>& labels);

        dataset(const std::filesystem::path& directory, const std::vector<std::string>& labels);

        std::tuple<dataset, dataset> sample(float ratio = 0.8);
    };

    std::tuple<cv::Mat,cv::Mat> format_data(const dataset& data);
}

#endif //SVM_H
