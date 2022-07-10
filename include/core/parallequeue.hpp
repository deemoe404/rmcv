//
// Created by yaione on 3/13/22.
//

#ifndef RM_STANDARD2022_PARALLEQUEUE_HPP
#define RM_STANDARD2022_PARALLEQUEUE_HPP

#include <mutex>
#include <condition_variable>
#include <deque>
#include <queue>
#include <memory>

namespace rm {
    template<typename DATATYPE, typename SEQUENCE = std::deque<DATATYPE>>
    class ParallelQueue {
    private:
        std::condition_variable m_cond;
        std::queue<DATATYPE, SEQUENCE> m_data;
        mutable std::mutex m_mutex;

    public:
        ParallelQueue() = default;

        ParallelQueue(ParallelQueue &&) = delete;

        ~ParallelQueue() = default;

        ParallelQueue &operator=(const ParallelQueue &) = delete;

        bool Empty() const {
            std::lock_guard<std::mutex> lg(m_mutex);
            return m_data.empty();
        }

        ParallelQueue(const ParallelQueue &other) {
            std::lock_guard<std::mutex> lg(other.m_mutex);
            m_data = other.m_data;
        }

        void push(const DATATYPE &data) {
            std::lock_guard<std::mutex> lg(m_mutex);
            m_data.push(data);
            m_cond.notify_one();
        }

        void push(DATATYPE &&data) {
            std::lock_guard<std::mutex> lg(m_mutex);
            m_data.push(std::move(data));
            m_cond.notify_one();
        }

        std::shared_ptr<DATATYPE> tryPop() {
            std::lock_guard<std::mutex> lg(m_mutex);
            if (m_data.empty()) return {};
            auto res = std::make_shared<DATATYPE>(m_data.front());
            m_data.pop();
            return res;
        }

        std::shared_ptr<DATATYPE> pop() {
            std::unique_lock<std::mutex> lg(m_mutex);
            m_cond.wait(lg, [this] { return !m_data.empty(); });
            auto res = std::make_shared<DATATYPE>(std::move(m_data.front()));
            m_data.pop();
            return res;
        }
    };
}

#endif //RM_STANDARD2022_PARALLEQUEUE_HPP
