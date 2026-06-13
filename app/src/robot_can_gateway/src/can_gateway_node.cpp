#include "robot_can_gateway/can_gateway_node.hpp"
#include "robot_can_gateway/can_protocol.hpp"

#include <chrono>

namespace robot_can_gateway
{

CanGatewayNode::CanGatewayNode(const std::string &can_iface)
	: Node("can_gateway")
{
	// Declare a parameter so the CAN interface name can be overridden
	// via launch file or command-line --ros-args.
	this->declare_parameter("can_interface", can_iface);

	std::string ifname = this->get_parameter("can_interface").as_string();

	if (!can_iface_.open(ifname)) {
		RCLCPP_ERROR(get_logger(), "Failed to open CAN interface '%s': %s",
			ifname.c_str(), can_iface_.get_last_error().c_str());
		rclcpp::shutdown();
		return;
	}

	RCLCPP_INFO(get_logger(), "Opened CAN interface '%s'", ifname.c_str());

	// Zero all frame counters
	for (auto &c : frame_counts_) {
		c.store(0, std::memory_order_relaxed);
	}

	// 1 Hz statistics timer
	this->create_wall_timer(std::chrono::seconds(1),
		std::bind(&CanGatewayNode::log_statistics, this));
}

CanGatewayNode::~CanGatewayNode()
{
	running_.store(false, std::memory_order_relaxed);
	if (can_thread_.joinable()) {
		can_thread_.join();
	}
	can_iface_.close();
}

void CanGatewayNode::start()
{
	running_.store(true, std::memory_order_relaxed);
	can_thread_ = std::thread(&CanGatewayNode::can_read_loop, this);
}

void CanGatewayNode::can_read_loop()
{
	struct can_frame frame;

	while (running_.load(std::memory_order_relaxed)) {
		if (!can_iface_.read(&frame)) {
			RCLCPP_ERROR(get_logger(), "CAN read error: %s",
				can_iface_.get_last_error().c_str());
			break;
		}

		// Update per-ID counters atomically
		switch (frame.can_id & CAN_SFF_MASK) {
		case 0x201: frame_counts_[0].fetch_add(1, std::memory_order_relaxed); break;
		case 0x202: frame_counts_[1].fetch_add(1, std::memory_order_relaxed); break;
		case 0x203: frame_counts_[2].fetch_add(1, std::memory_order_relaxed); break;
		case 0x204: frame_counts_[3].fetch_add(1, std::memory_order_relaxed); break;
		case 0x205: frame_counts_[4].fetch_add(1, std::memory_order_relaxed); break;
		case 0x101: frame_counts_[5].fetch_add(1, std::memory_order_relaxed); break;
		default:    frame_counts_[6].fetch_add(1, std::memory_order_relaxed); break;
		}
		frame_counts_[7].fetch_add(1, std::memory_order_relaxed);
	}

	RCLCPP_INFO(get_logger(), "CAN read thread exiting");
}

void CanGatewayNode::log_statistics()
{
	auto total = frame_counts_[7].load(std::memory_order_relaxed);
	auto c201  = frame_counts_[0].load(std::memory_order_relaxed);
	auto c202  = frame_counts_[1].load(std::memory_order_relaxed);
	auto c203  = frame_counts_[2].load(std::memory_order_relaxed);
	auto c204  = frame_counts_[3].load(std::memory_order_relaxed);
	auto c205  = frame_counts_[4].load(std::memory_order_relaxed);
	auto c101  = frame_counts_[5].load(std::memory_order_relaxed);
	auto other = frame_counts_[6].load(std::memory_order_relaxed);

	RCLCPP_INFO(get_logger(),
		"frames: total=%lu  201(M1M2)=%lu  202(M3M4)=%lu  "
		"203(Accel)=%lu  204(Gyro)=%lu  205(Mag)=%lu  "
		"101(Cmd)=%lu  other=%lu",
		total, c201, c202, c203, c204, c205, c101, other);
}

} // namespace robot_can_gateway

// ---- main ----

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<robot_can_gateway::CanGatewayNode>();
	node->start();

	if (rclcpp::ok()) {
		rclcpp::spin(node);
	}
	rclcpp::shutdown();
	return 0;
}
