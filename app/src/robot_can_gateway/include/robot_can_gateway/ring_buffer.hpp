#pragma once

#include <atomic>
#include <cstddef>
#include <linux/can.h>

namespace robot_can_gateway
{

// SPSC (Single Producer Single Consumer) lock-free ring buffer.
// Thread 1 (CAN reader) calls push(). Thread 2 (executor) calls pop_all().
// 256 entries = ~4KB, enough for ~300ms of burst at 770fps.
class CanRingBuffer
{
public:
	static constexpr size_t CAPACITY = 256;

	CanRingBuffer()
		: head_(0)
		, tail_(0)
	{
	}

	// Push one frame. Called from CAN read thread only.
	// Returns false if full (frame dropped).
	bool push(const struct can_frame &frame)
	{
		size_t h = head_.load(std::memory_order_relaxed);
		size_t next = (h + 1) % CAPACITY;
		if (next == tail_.load(std::memory_order_acquire))
			return false;	// full
		buf_[h] = frame;
		head_.store(next, std::memory_order_release);
		return true;
	}

	// Pop up to `max` frames into `out`. Returns number of frames popped.
	// Called from executor thread only.
	size_t pop_all(struct can_frame *out, size_t max)
	{
		size_t t = tail_.load(std::memory_order_relaxed);
		size_t h = head_.load(std::memory_order_acquire);
		size_t count = 0;

		while (t != h && count < max) {
			out[count++] = buf_[t];
			t = (t + 1) % CAPACITY;
		}

		if (count > 0)
			tail_.store(t, std::memory_order_release);

		return count;
	}

	// Number of frames currently in the buffer.
	size_t size() const
	{
		size_t h = head_.load(std::memory_order_acquire);
		size_t t = tail_.load(std::memory_order_acquire);
		return (h >= t) ? (h - t) : (CAPACITY - t + h);
	}

private:
	struct can_frame buf_[CAPACITY];
	std::atomic<size_t> head_;	// Thread1 writes
	std::atomic<size_t> tail_;	// Thread2 writes
};

} // namespace robot_can_gateway
