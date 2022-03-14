//
// Created by yaione on 3/12/22.
//

#include "rmcv/mlp/training.h"

namespace rm::mlp {
    void CaptureImage(const string &path, Mat &input, rm::mlp::CaptureType type, int label) {
        int tickCount = (int) ((double) cv::getTickCount() / cv::getTickFrequency());
        std::string fullPath, filePath;
        if (type == rm::mlp::CAPTURE_MIXED) {
            fullPath = path;
        } else if (type == rm::mlp::CAPTURE_LABEL) {
            fullPath = rm::PathCombine(path, rm::int2str(label) + "/");
        }
        mkdir(fullPath.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
        filePath = rm::PathCombine(fullPath, rm::int2str(tickCount) + ".jpg");
        cv::imwrite(filePath, input);
    }

    void Labeling(const std::string &path, int fileCounts) {
        cv::Mat image;
        while (fileCounts--) {
            std::string filename = int2str(fileCounts) + ".jpg";
            image = cv::imread(path + filename);
            cv::imshow("Plz input label (0~9).", image);
            std::string key = int2str(cv::waitKey(0) - 48) + "/" + filename;
            cv::imwrite(rm::PathCombine(path, key), image);
        }
    }

    void
    TrainMLP(const std::string &path, const std::vector<int> &labels, int imgCount, cv::Ptr<cv::ml::ANN_MLP> &model) {
        cv::Mat first = cv::imread(rm::PathCombine(path, rm::int2str(labels[0]) + "/1.jpg"));
        int images = imgCount * (int) labels.size(), pixels = first.cols * first.rows;
        float imgData[images][pixels];
        float labelData[images][labels.size()];

        for (int i = 0; i < labels.size(); i++) {
            for (int j = 0; j < imgCount; j++) {
                cv::Mat tmp = cv::imread(rm::PathCombine(path, rm::int2str(labels[i]) + "/" + rm::int2str(j) + ".jpg"));
                cv::cvtColor(tmp, tmp, cv::COLOR_BGR2GRAY);
                cv::Mat row = tmp.reshape(0, 1);
                for (int k = 0; k < pixels; k++) {
                    imgData[j + i * imgCount][k] = row.at<uchar>(0, k);
                }

                for (int l = 0; l < labels.size(); l++) {
                    if (l == i) {
                        labelData[j + i * imgCount][l] = 1;
                    } else if (l != i) {
                        labelData[j + i * imgCount][l] = -1;
                    }
                }
            }
        }

        cv::Mat imgMat(images, pixels, CV_32FC1, imgData);
        cv::Mat labelMat(images, (int) labels.size(), CV_32FC1, labelData);

        model = cv::ml::ANN_MLP::create();
        cv::Mat layerSizes = (Mat_<int>(1, 5) << pixels, pixels / 2, pixels / 4, sqrt(pixels / 4), labels.size());
        model->setLayerSizes(layerSizes);
        model->setActivationFunction(cv::ml::ANN_MLP::SIGMOID_SYM, 1.0, 1.0);
        model->setTrainMethod(cv::ml::ANN_MLP::BACKPROP, 0.1, 0.1);
        model->setTermCriteria(cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 5000, 0.01));
        cv::Ptr<cv::ml::TrainData> trainData = cv::ml::TrainData::create(imgMat, cv::ml::ROW_SAMPLE, labelMat);
        model->train(trainData);
    }
}
