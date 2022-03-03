//
// Created by yaione on 2/26/2022.
//

#include "core.h"

namespace rm {
    LightBar::LightBar(cv::RotatedRect ellipse) {
        VerticesRectify(ellipse, vertices);

        center = ellipse.center;
        size = ellipse.size;
        angle = ellipse.angle > 90 ? ellipse.angle - 90 : ellipse.angle + 90;
    }

    Armour::Armour(std::vector<rm::LightBar> lightBars) {
        if (lightBars.size() != 2) {
            throw std::runtime_error("Armour must be initialized with 2 rm::LightBar (s).");
        }

        // sort light bars left to right
        std::sort(lightBars.begin(), lightBars.end(),
                  [](rm::LightBar lightBar_1, rm::LightBar lightBar_2) {
                      return lightBar_1.center.x < lightBar_2.center.x;
                  });

        int i = 0;
        for (auto lightBar: lightBars) {
            vertices[i++] = lightBar.vertices[i];
            vertices[i++] = lightBar.vertices[i];
        }
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
