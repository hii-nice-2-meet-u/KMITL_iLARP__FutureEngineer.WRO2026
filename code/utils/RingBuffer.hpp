#pragma once

#include <array>
#include <cstddef>
#include <utility>

template <typename T, std::size_t Capacity>
class RingBuffer
{
public:
    static_assert(Capacity > 0, "RingBuffer capacity must be greater than 0");

    void push(const T& value)
    {
        buffer_[head_] = value;
        advance();
    }

    void push(T&& value)
    {
        buffer_[head_] = std::move(value);
        advance();
    }

    bool latest(T& out) const
    {
        if (size_ == 0) {
            return false;
        }

        const std::size_t index =
            (head_ + Capacity - 1) % Capacity;

        out = buffer_[index];
        return true;
    }

    std::size_t size() const noexcept
    {
        return size_;
    }

    constexpr std::size_t capacity() const noexcept
    {
        return Capacity;
    }

    bool empty() const noexcept
    {
        return size_ == 0;
    }

private:
    void advance()
    {
        head_ = (head_ + 1) % Capacity;

        if (size_ < Capacity) {
            ++size_;
        }
    }

    std::array<T, Capacity> buffer_{};

    std::size_t head_{0};
    std::size_t size_{0};
};