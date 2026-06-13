#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include "robot_can_gateway/can_interface.hpp"
#include "robot_can_gateway/kinematics.hpp"

namespace robot_can_gateway
{

class CanGatewayNode : public rclcpp::Node
{
public:
	explicit CanGatewayNode(const std::string &can_iface = "can0");
	~CanGatewayNode();

	void start();

private:
	// ── CAN I/O ──────────────────────────────────────────────
	void can_read_loop();		// blocking read thread
	void can_tx_timer_callback();	// 100Hz: send cached /cmd_vel as 0x101

	// ── ROS2 subscriptions ───────────────────────────────────
	void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
	void estop_callback(const std_msgs::msg::Bool::SharedPtr msg);

	// ── Statistics ───────────────────────────────────────────
	void log_statistics();		// 1Hz timer

	// ── CAN interface ────────────────────────────────────────
	CanInterface can_iface_;

	// ── Publishers ───────────────────────────────────────────
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr      imu_pub_;

	// ── Subscriptions ────────────────────────────────────────
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr       estop_sub_;

	// ── Timers ───────────────────────────────────────────────
	rclcpp::TimerBase::SharedPtr can_tx_timer_;	// 100Hz
	rclcpp::TimerBase::SharedPtr stats_timer_;	// 1Hz

	// ── Cached cmd_vel (written by callback, read by timer) ──
	std::atomic<double> cached_vx_;
	std::atomic<double> cached_wz_;
	std::atomic<int64_t> last_cmd_vel_ns_;	// timestamp of last cmd_vel

	// ── Odometry state (only touched by can_read_loop) ───────
	OdometryPose odom_pose_;
	int16_t cached_m1_speed_;		// latest M1 (updated by 0x201)
	int16_t cached_m2_speed_;		// latest M2 (updated by 0x201)
	int16_t cached_m3_speed_;		// latest M3 (updated by 0x202)
	int16_t cached_m4_speed_;		// latest M4 (updated by 0x202)
	double  cached_accel_x_;		// from 0x203, consumed by 0x204
	double  cached_accel_y_;
	double  cached_accel_z_;

	// ── Frame counters ───────────────────────────────────────
	std::atomic<uint64_t> frame_counts_[8];	// [0]=0x201...[5]=0x101,[6]=other,[7]=total

	// ── Thread control ───────────────────────────────────────
	std::atomic<bool> running_;
	std::thread can_thread_;
};

} // namespace robot_can_gateway
