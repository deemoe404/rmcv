//
// Created by yaione on 2/26/2022.
//

#include "rmcv/core/core.h"

namespace rm {
    LightBar::LightBar(cv::RotatedRect box, float angle) : angle(angle) {
        VerticesRectify(box, vertices, RECT_TALL);

        center = box.center;
        size.height = max(box.size.height, box.size.width);
        size.width = min(box.size.height, box.size.width);
    }

    Armour::Armour(std::vector<rm::LightBar> lightBars, rm::ArmourType type) : armourType(type) {
        if (lightBars.size() != 2) {
            throw std::runtime_error("Armour must be initialized with 2 rm::LightBar (s).");
        }

        // sort light bars left to right
        std::sort(lightBars.begin(), lightBars.end(), [](rm::LightBar lightBar1, rm::LightBar lightBar2) {
            return lightBar1.center.x < lightBar2.center.x;
        });

        int i = 0, j = 3;
        for (auto lightBar: lightBars) {
            vertices[i++] = lightBar.vertices[j--];
            vertices[i++] = lightBar.vertices[j--];
        }

        float distanceL = rm::PointDistance(vertices[0], vertices[1]);
        float distanceR = rm::PointDistance(vertices[2], vertices[3]);
        int offsetL = (int) round((distanceL / 0.44f - distanceL) / 2);
        int offsetR = (int) round((distanceR / 0.44f - distanceR) / 2);
        ExCord(vertices[0], vertices[1], offsetL, iconBox[0], iconBox[1]);
        ExCord(vertices[3], vertices[2], offsetR, iconBox[3], iconBox[2]);

        box = cv::boundingRect(std::vector<cv::Point>({vertices[0], vertices[1], vertices[2], vertices[3]}));
    }

    template<typename DATATYPE, typename SEQUENCE>
    ParallelQueue<DATATYPE, SEQUENCE>::ParallelQueue(const ParallelQueue &other) {
        std::lock_guard<std::mutex> lg(other.m_mutex);
        m_data = other.m_data;
    }

    template<typename DATATYPE, typename SEQUENCE>
    void ParallelQueue<DATATYPE, SEQUENCE>::push(const DATATYPE &data) {
        std::lock_guard<std::mutex> lg(m_mutex);
        m_data.push(data);
        m_cond.notify_one();
    }

    template<typename DATATYPE, typename SEQUENCE>
    void ParallelQueue<DATATYPE, SEQUENCE>::push(DATATYPE &&data) {
        std::lock_guard<std::mutex> lg(m_mutex);
        m_data.push(std::move(data));
        m_cond.notify_one();
    }

    template<typename DATATYPE, typename SEQUENCE>
    std::shared_ptr<DATATYPE> ParallelQueue<DATATYPE, SEQUENCE>::tryPop() {
        std::lock_guard<std::mutex> lg(m_mutex);
        if (m_data.empty()) return {};
        auto res = std::make_shared<DATATYPE>(m_data.front());
        m_data.pop();
        return res;
    }

    template<typename DATATYPE, typename SEQUENCE>
    std::shared_ptr<DATATYPE> ParallelQueue<DATATYPE, SEQUENCE>::pop() {
        std::unique_lock<std::mutex> lg(m_mutex);
        m_cond.wait(lg, [this] { return !m_data.empty(); });
        auto res = std::make_shared<DATATYPE>(std::move(m_data.front()));
        m_data.pop();
        return res;
    }
}
