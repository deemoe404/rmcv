//
// Created by Sam Cheung on 7/23/24.
//

#include <svm.h>

namespace rm::svm
{
    dataset::dataset(const std::vector<std::string>& labels): labels(labels)
    {
    }

    dataset::dataset(const std::filesystem::path& directory,
                     const std::vector<std::string>& labels): labels(labels)
    {
        for (int i = 0; i < labels.size(); i++)
        {
            this->insert({i, utils::read_image_recursive(directory / labels[i])});
        }
    }

    std::tuple<dataset, dataset> dataset::sample(const float ratio)
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

    std::tuple<cv::Mat, cv::Mat> format_data(const dataset& data)
    {
        cv::Mat samples, responses;
        for (const auto& [index, images] : data)
        {
            for (const auto& image : images)
            {
                samples.push_back(image);
                responses.push_back(index);
            }
        }

        return {samples, responses};
    }
}
