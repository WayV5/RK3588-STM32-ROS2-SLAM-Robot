#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "robot_can_gateway/can_interface.hpp"
#include "robot_can_gateway/kinematics.hpp"
#include "robot_can_gateway/ring_buffer.hpp"

namespace robot_can_gateway
{

class CanGatewayNode : public rclcpp::Node
{
public:
	explicit CanGatewayNode(const std::string &can_iface = "can0");
	~CanGatewayNode();

	void start();

private:
	// Thread 1: blocking read → ringbuf push
	void can_read_loop();

	// Thread 2: 100Hz timer → ringbuf pop_all → decode → publish
	void process_telemetry();

	// /cmd_vel subscription → cache twist
	void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

	// /emergency_stop subscription → write 0x102 immediately
	void estop_callback(const std_msgs::msg::Bool::SharedPtr msg);

	// 100Hz: send cached cmd_vel as 0x101
	void can_tx_timer_callback();

	// 1Hz: publish /diagnostics + log rates
	void publish_diagnostics();

	CanInterface can_iface_;
	CanRingBuffer ringbuf_;

	// Publishers
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr      imu_pub_;
	rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	// Subscriptions
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr       estop_sub_;

	// Timers
	rclcpp::TimerBase::SharedPtr process_timer_;	// 100Hz
	rclcpp::TimerBase::SharedPtr can_tx_timer_;	// 100Hz
	rclcpp::TimerBase::SharedPtr stats_timer_;	// 1Hz

	// Cached cmd_vel
	std::atomic<double> cached_vx_;
	std::atomic<double> cached_wz_;
	std::atomic<int64_t> last_cmd_vel_ns_;

	// Odometry integration (Thread 2 only)
	OdometryPose odom_pose_;
	double cached_accel_x_;
	double cached_accel_y_;
	double cached_accel_z_;

	// Frame counters + per-second rate tracking
	std::atomic<uint64_t> frame_counts_[8];
	uint64_t              prev_counts_[8];	// for delta-per-second
	std::atomic<int64_t>  last_heartbeat_ns_;	// 0x206 arrival timestamp
	std::atomic<int64_t>  last_can_rx_ns_;		// any frame arrival timestamp

	std::atomic<bool> running_;
	std::thread can_thread_;
};

} // namespace robot_can_gateway
