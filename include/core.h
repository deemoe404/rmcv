//
// Created by yaione on 2/26/2022.
//

#ifndef RM_STANDARD2022_CORE_H
#define RM_STANDARD2022_CORE_H

#include "opencv2/opencv.hpp"
#include "utils.h"
#include <mutex>
#include <condition_variable>
#include <deque>
#include <queue>
#include <memory>

namespace rm {
    enum ForceType {
        FORCE_HERO = 0,
        FORCE_STANDARD = 1,
        FORCE_ENGINEER = 2,
        FORCE_AERIAL = 3,
        FORCE_SENTRY = 4
    };

    enum CampType {
        CAMP_RED = 0,
        CAMP_BLUE = 1
    };

    class LightBar {
    public:
        cv::Point2f vertices[4];
        cv::Point2f center;
        cv::Size2f size;
        float angle;

        explicit LightBar(cv::RotatedRect ellipse);
    };

    class Armour {
    public:
        cv::Point2f vertices[4];
        cv::Point2f center;
        cv::Size2f size;
        cv::Mat rvecs;
        cv::Mat tvecs;
        float area = 0;
        float rank = -1;

        explicit Armour(std::vector<rm::LightBar> lightBars);
    };

    template<typename DATATYPE, typename SEQUENCE = std::deque<DATATYPE>>
    class ParallelQueue {
    public:
        ParallelQueue() = default;
        ParallelQueue(const ParallelQueue &other);
        ParallelQueue(ParallelQueue &&) = delete;
        ParallelQueue &operator=(const ParallelQueue &) = delete;

        ~ParallelQueue() = default;

        void push(const DATATYPE &data);
        void push(DATATYPE &&data);

        std::shared_ptr<DATATYPE> tryPop();
        std::shared_ptr<DATATYPE> pop();

    private:
        std::queue<DATATYPE, SEQUENCE> m_data;
        mutable std::mutex m_mutex;
        std::condition_variable m_cond;
    };

}

#endif //RM_STANDARD2022_CORE_H
