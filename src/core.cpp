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
        std::sort(lightBars.begin(), lightBars.end(), [](rm::LightBar lightBar_1, rm::LightBar lightBar_2) {
            return lightBar_1.center.x < lightBar_2.center.x;
        });

        int i = 0;
        for (auto lightBar: lightBars) {
            vertices[i] = lightBar.vertices[i];
            i++;
            vertices[i] = lightBar.vertices[i];
            i++;
        }
//        printf("");

        box.x = (int) std::min(vertices[0].x, vertices[1].x);
        box.y = (int) std::min(vertices[1].x, vertices[2].x);
        box.width = (int) (std::max(std::max(vertices[0].x, vertices[1].x), std::max(vertices[2].x, vertices[3].x)) -
                           std::min(std::min(vertices[0].x, vertices[1].x), std::min(vertices[2].x, vertices[3].x)));
        box.height = (int) (std::max(std::max(vertices[0].y, vertices[1].y), std::max(vertices[2].y, vertices[3].y)) -
                            std::min(std::min(vertices[0].y, vertices[1].y), std::min(vertices[2].y, vertices[3].y)));
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
