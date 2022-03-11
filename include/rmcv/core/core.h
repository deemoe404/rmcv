//
// Created by yaione on 2/26/2022.
//

#ifndef RM_STANDARD2022_CORE_H
#define RM_STANDARD2022_CORE_H

#include "opencv2/opencv.hpp"
#include "utils.h"
#include "serialport.h"
#include "daheng/daheng.h"
#include <mutex>
#include <condition_variable>
#include <deque>
#include <queue>
#include <memory>

namespace rm {
    enum ArmourType {
        ARMOUR_BIG = 0, ARMOUR_SMALL = 1
    };

    enum ForceType {
        FORCE_HERO = 0, FORCE_STANDARD = 1, FORCE_ENGINEER = 2, FORCE_AERIAL = 3, FORCE_SENTRY = 4
    };

    enum CampType {
        CAMP_RED = 0, CAMP_BLUE = 1
    };

    class LightBar {
    public:
        cv::Point vertices[4]; // Four vertices around light bar
        cv::Point center;
        cv::Size2f size;
        float angle;

        explicit LightBar(cv::RotatedRect box, float angle);
    };

    class Armour {
    public:
        cv::Point vertices[4]; // Vertices around two light bars
        cv::Point iconBox[4];  // Vertices around icon
        cv::Rect box;          // Bounding rect
        cv::Mat rvecs;
        cv::Mat tvecs;
        rm::ForceType forceType = rm::FORCE_STANDARD;
        rm::ArmourType armourType = rm::ARMOUR_SMALL;
        double airTime = 0;
        float pitch = 0;
        float yaw = 0;
        char rank = -1;

        explicit Armour(std::vector<rm::LightBar> lightBars, rm::ArmourType type = rm::ARMOUR_SMALL);
    };

    template<typename DATATYPE, typename SEQUENCE = std::deque<DATATYPE>>
    class ParallelQueue {
    private:
        std::condition_variable m_cond;
        std::queue<DATATYPE, SEQUENCE> m_data;
        mutable std::mutex m_mutex;

    public:
        ParallelQueue() = default;

        ParallelQueue(const ParallelQueue &other);

        ParallelQueue(ParallelQueue &&) = delete;

        ~ParallelQueue() = default;

        ParallelQueue &operator=(const ParallelQueue &) = delete;

        void push(const DATATYPE &data);

        void push(DATATYPE &&data);

        std::shared_ptr<DATATYPE> tryPop();

        std::shared_ptr<DATATYPE> pop();
    };

}

#endif //RM_STANDARD2022_CORE_H
