#pragma once
#include <stdint.h>
#include "dlx_locked_cell.hpp"
#include "dlx_queue.hpp"

namespace dlx
{
template <typename T>
class SampleSlot
{
    struct Published
    {
        T sample{};
        uint32_t sequence{0u};
    };
    LockedCell<Published> value{};

public:
    void publish(const T &sample)
    {
        value.update([&](Published &next) {
            next.sample = sample;
            if (++next.sequence == 0u) ++next.sequence;
        });
    }

    bool read(T &sample) const
    {
        const Published current = value.load();
        if (current.sequence == 0u) return false;
        sample = current.sample;
        return true;
    }

    bool readNew(T &sample, uint32_t &sequence) const
    {
        const Published current = value.load();
        if (current.sequence == 0u || current.sequence == sequence) return false;
        sample = current.sample;
        sequence = current.sequence;
        return true;
    }
};

template <typename T, uint16_t Capacity>
class SampleStream
{
    static_assert(Capacity > 0u && sizeof(T) * Capacity <= 65535u, "Invalid sample stream capacity");
    uint8_t storage[sizeof(T) * Capacity]{};
    ByteBuffer buffer{storage, static_cast<uint16_t>(sizeof(storage))};
    SPSCQueue<T> queue{buffer};
    std::atomic<uint32_t> droppedCount{0u};

public:
    bool publish(const T &sample)
    {
        if (queue.push(sample)) return true;
        droppedCount.fetch_add(1u, std::memory_order_relaxed);
        return false;
    }
    bool read(T &sample) { return queue.pop(sample); }
    uint16_t size() const { return queue.size(); }
    uint32_t dropped() const { return droppedCount.load(std::memory_order_relaxed); }
};
}
