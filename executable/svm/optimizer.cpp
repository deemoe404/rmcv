//
// Created by Sam Cheung on 7/19/24.
//

#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>

#include <string>
#include <algorithm>
#include <filesystem>
#include <random>

auto list_directory_recursive(const std::filesystem::path& directory,
                              const std::vector<std::string>& extension_whitelist = {})
{
    std::vector<std::filesystem::path> files;
    for (const auto& p : std::filesystem::recursive_directory_iterator(directory))
    {
        if (std::filesystem::is_directory(p)) continue;
        if (!extension_whitelist.empty())
        {
            auto extension = p.path().extension().string();
            auto head = extension_whitelist.begin(),
                 tail = extension_whitelist.end();
            if (auto iterator = find(head, tail, extension); iterator != tail)
                files.push_back(p.path());
        }
        else files.push_back(p.path());
    }
    return files;
}

auto read_image_recursive(const std::filesystem::path& directory)
{
    std::vector<cv::Mat> images;
    const auto files = list_directory_recursive(directory, {".jpg"});
    for_each(files.begin(), files.end(), [&images](const std::filesystem::path& p)
    {
        cv::Mat image = cv::imread(p.string());
        resize(image, image, {20, 20}, 0, 0, cv::INTER_LINEAR);
        image = image.reshape(1, 1);
        image.convertTo(image, CV_32FC1);
        images.push_back(image);
    });
    return images;
}

class dataset : public std::map<int, std::vector<cv::Mat>>
{
public:
    std::vector<std::string> labels;

    explicit dataset(const std::vector<std::string>& labels): labels(labels)
    {
    }

    dataset(const std::filesystem::path& directory, const std::vector<std::string>& labels): labels(labels)
    {
        for (int i = 0; i < labels.size(); i++)
        {
            this->insert({i, read_image_recursive(directory / labels[i])});
        }
    }

    std::tuple<dataset, dataset> sample(const float ratio = 0.8)
    {
        dataset head(labels), tail(labels);
        for (auto& [index, images] : *this)
        {
            std::shuffle(images.begin(), images.end(),
                         std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count()));
            const auto split = static_cast<int>(static_cast<float>(images.size()) * ratio);
            head.insert({index, {images.begin(), images.begin() + split}});
            tail.insert({index, {images.begin() + split, images.end()}});
        }
        return {head, tail};
    }
};

int main()
{
    dataset rm_labels("images/20240719/", {"1", "2", "3", "4", "5", "Sentry", "Negtive"});

    float correct = 0, total = 0;
    for (int i = 0; i < 100; i++)
    {
        auto [training_set, validation_set] = rm_labels.sample(0.6);

        cv::Mat samples, responses;
        for (const auto& [index, images] : training_set)
        {
            for (const auto& image : images)
            {
                samples.push_back(image);
                responses.push_back(index);
            }
        }

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
        std::cout << i << " Accuracy: " << correct / total * 100 << "%, train time: " << train_time <<
            "s, inference time: " << inference_time / total << "us." << std::endl;
    }
    std::cout << "Total Accuracy: " << correct / total * 100 << "%" << std::endl;

    // svm->save("../svm.xml");

    return 0;
}
