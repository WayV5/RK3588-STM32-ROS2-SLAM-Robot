#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "robot_can_gateway/can_interface.hpp"

namespace robot_can_gateway
{

class CanGatewayNode : public rclcpp::Node
{
public:
	explicit CanGatewayNode(const std::string &can_iface = "can0");
	~CanGatewayNode();

	// Start the CAN read thread. Safe to call after construction.
	void start();

private:
	// CAN read thread function: blocking read → decode → increment counters.
	void can_read_loop();

	// Timer callback (1 Hz): logs per-ID frame counts via RCLCPP_INFO.
	void log_statistics();

	CanInterface can_iface_;

	// Per-CAN-ID counters: [0]=0x201, [1]=0x202, [2]=0x203,
	// [3]=0x204, [4]=0x205, [5]=0x101, [6]=other, [7]=total.
	std::atomic<uint64_t> frame_counts_[8];

	std::atomic<bool> running_;
	std::thread can_thread_;
};

} // namespace robot_can_gateway
