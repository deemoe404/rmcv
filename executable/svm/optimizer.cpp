//
// Created by Sam Cheung on 7/19/24.
//

#include "rmcv.h"

int main()
{
    rm::svm::dataset rm_labels("images/20240719/", {"1", "2", "3", "4", "5", "Sentry", "Negtive"});

    float correct = 0, total = 0;

    auto [training_set, validation_set] = rm_labels.sample(0.6);
    auto [samples, responses] = format_data(training_set);

    const cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
    svm->setType(cv::ml::SVM::C_SVC);
    svm->setKernel(cv::ml::SVM::LINEAR);
    svm->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 1000, 1e-3));

    int64 tick = cv::getTickCount();
    svm->trainAuto(samples, cv::ml::ROW_SAMPLE, responses);

    const double train_time = static_cast<double>(cv::getTickCount() - tick) / cv::getTickFrequency();
    tick = cv::getTickCount();

    for (const auto& [index, images] : validation_set)
    {
        for (auto& image : images)
        {
            if (index == static_cast<int>(svm->predict(image))) correct++;
            total++;
        }
    }

    const double inference_time = static_cast<double>(cv::getTickCount() - tick) / cv::getTickFrequency() * 1000 * 1000;
    std::cout << " Accuracy: " << correct / total * 100 << "%, train time: " << train_time <<
        "s, inference time: " << inference_time / total << "us." << std::endl;

    std::cout << "Total Accuracy: " << correct / total * 100 << "%" << std::endl;

    svm->save("svm.xml");

    return 0;
}
