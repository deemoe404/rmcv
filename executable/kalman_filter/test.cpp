//
// Created by Sam Cheung on 7/24/24.
//

#include "rmcv.h"

int main()
{
    cv::KalmanFilter KF(6, 6, 0);

    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, cv::Scalar::all(5e-5));
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(0.5));
    setIdentity(KF.errorCovPost, cv::Scalar::all(0.05));

    cv::Mat measurement = cv::Mat::zeros(6, 1, CV_32F);

    KF.transitionMatrix = (cv::Mat_<float>(6, 6) <<
        1, 0, 0, 1, 0, 0,
        0, 1, 0, 0, 1, 0,
        0, 0, 1, 0, 0, 1,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1);

    bool firstKF = true;

    auto function = [](const float& t) -> float
    {
        auto noise = [
                generator = std::default_random_engine(cv::getTickCount()),
                distribution = std::normal_distribution<float>(0, 10)]() mutable
        {
            return distribution(generator);
        };
        return 2.0f * t + noise();
    };

    const auto start_time = cv::getTickCount();

    int i = 100;
    float t_last = 0;
    while (i--)
    {
        const auto dtick = static_cast<float>(cv::getTickCount() - start_time);
        const float t = dtick / static_cast<float>(cv::getTickFrequency()) * 1000.0f;

        if (firstKF == true)
        {
            measurement.at<float>(0) = function(t);
            measurement.at<float>(1) = function(t);
            measurement.at<float>(2) = function(t);

            KF.correct(measurement);
            firstKF = false;

            t_last = t;
        }
        else
        {
            KF.predict();

            const auto dt = t - t_last;
            const auto value = function(t);

            KF.transitionMatrix.at<float>(0, 3) = dt;
            KF.transitionMatrix.at<float>(1, 4) = dt;
            KF.transitionMatrix.at<float>(2, 5) = dt;

            measurement.at<float>(3) = (value - measurement.at<float>(0)) / dt;
            measurement.at<float>(4) = (value - measurement.at<float>(1)) / dt;
            measurement.at<float>(5) = (value - measurement.at<float>(2)) / dt;

            measurement.at<float>(0) = value;
            measurement.at<float>(1) = value;
            measurement.at<float>(2) = value;

            t_last = t;

            KF.correct(measurement);

            auto test = cv::Mat(KF.transitionMatrix * KF.statePost);
            std::cout << test << std::endl;
            std::cout << 2 * t << std::endl;
        }
    }
}
