// Fixed-capacity FIFO with no heap use, for the firmware and the navigator.
#pragma once

#include <array>
#include <cstddef>

namespace gins {

template <typename T, std::size_t N>
class RingQueue {
public:
    bool push(const T& v) {
        if (size_ == N) {
            return false;
        }
        buf_[(head_ + size_) % N] = v;
        ++size_;
        return true;
    }
    const T& front() const { return buf_[head_]; }
    void pop() {
        head_ = (head_ + 1) % N;
        --size_;
    }
    bool empty() const { return size_ == 0; }
    bool full() const { return size_ == N; }
    std::size_t size() const { return size_; }
    static constexpr std::size_t capacity() { return N; }
    void clear() {
        head_ = 0;
        size_ = 0;
    }

private:
    std::array<T, N> buf_{};
    std::size_t head_ = 0;
    std::size_t size_ = 0;
};

}  // namespace gins
