#include "robot_can_gateway/can_gateway_node.hpp"
#include "robot_can_gateway/can_protocol.hpp"

#include <chrono>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>

namespace robot_can_gateway
{

CanGatewayNode::CanGatewayNode(const std::string &can_iface)
	: Node("can_gateway")
	, odom_pose_{0.0, 0.0, 0.0}
	, cached_m1_speed_(0)
	, cached_m2_speed_(0)
	, cached_m3_speed_(0)
	, cached_m4_speed_(0)
	, cached_accel_x_(0.0)
	, cached_accel_y_(0.0)
	, cached_accel_z_(0.0)
{
	this->declare_parameter("can_interface", can_iface);

	std::string ifname = this->get_parameter("can_interface").as_string();

	if (!can_iface_.open(ifname)) {
		RCLCPP_ERROR(get_logger(), "Failed to open CAN interface '%s': %s",
			ifname.c_str(), can_iface_.get_last_error().c_str());
		rclcpp::shutdown();
		return;
	}

	RCLCPP_INFO(get_logger(), "Opened CAN interface '%s'", ifname.c_str());

	// Frame counters
	for (auto &c : frame_counts_) {
		c.store(0, std::memory_order_relaxed);
	}

	// Cached cmd_vel defaults
	cached_vx_.store(0.0, std::memory_order_relaxed);
	cached_wz_.store(0.0, std::memory_order_relaxed);
	last_cmd_vel_ns_.store(0, std::memory_order_relaxed);

	// ── Publishers ──────────────────────────────────────────
	odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom",
		rclcpp::QoS(1).best_effort());
	imu_pub_  = this->create_publisher<sensor_msgs::msg::Imu>("/imu",
		rclcpp::QoS(5).best_effort());

	// ── Subscriptions ───────────────────────────────────────
	cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
		"/cmd_vel", rclcpp::QoS(10).reliable(),
		std::bind(&CanGatewayNode::cmd_vel_callback, this, std::placeholders::_1));

	estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
		"/emergency_stop", rclcpp::QoS(1).reliable(),
		std::bind(&CanGatewayNode::estop_callback, this, std::placeholders::_1));

	// ── Timers ──────────────────────────────────────────────
	can_tx_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
		std::bind(&CanGatewayNode::can_tx_timer_callback, this));

	stats_timer_ = this->create_wall_timer(std::chrono::seconds(1),
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

// ═══════════════════════════════════════════════════════════════
// CAN read thread
// ═══════════════════════════════════════════════════════════════

void CanGatewayNode::can_read_loop()
{
	struct can_frame frame;

	while (running_.load(std::memory_order_relaxed)) {
		if (!can_iface_.read(&frame)) {
			RCLCPP_ERROR(get_logger(), "CAN read error: %s",
				can_iface_.get_last_error().c_str());
			break;
		}

		frame_counts_[7].fetch_add(1, std::memory_order_relaxed);
		uint32_t id = frame.can_id & CAN_SFF_MASK;

		switch (id) {

		// ── 0x201: cache M1+M2, publish odometry with latest 4 wheels ─
		case 0x201: {
			frame_counts_[0].fetch_add(1, std::memory_order_relaxed);
			auto t = protocol::decode_motor_telemetry_1(frame);
			cached_m1_speed_ = t.motor_a.speed_mms;
			cached_m2_speed_ = t.motor_b.speed_mms;

			WheelSpeeds ws = {cached_m1_speed_, cached_m2_speed_,
			                  cached_m3_speed_, cached_m4_speed_};
			VehicleTwist vt = forward_kinematics(ws);
			odometry_integrate(odom_pose_, vt, 1.0 / 125.0);

			auto msg = nav_msgs::msg::Odometry();
			msg.header.stamp    = this->now();
			msg.header.frame_id = "odom";
			msg.child_frame_id  = "base_footprint";
			msg.pose.pose.position.x = odom_pose_.x;
			msg.pose.pose.position.y = odom_pose_.y;
			tf2::Quaternion q;
			q.setRPY(0, 0, odom_pose_.θ);
			msg.pose.pose.orientation.x = q.x();
			msg.pose.pose.orientation.y = q.y();
			msg.pose.pose.orientation.z = q.z();
			msg.pose.pose.orientation.w = q.w();
			msg.twist.twist.linear.x  = vt.v_x;
			msg.twist.twist.angular.z = vt.w_z;
			msg.pose.covariance[0]  = -1.0;
			msg.pose.covariance[7]  = -1.0;
			msg.pose.covariance[35] = -1.0;
			msg.twist.covariance[0] = -1.0;
			msg.twist.covariance[35]= -1.0;
			odom_pub_->publish(msg);
			break;
		}

		// ── 0x202: cache M3+M4, publish odometry with latest 4 wheels ─
		case 0x202: {
			frame_counts_[1].fetch_add(1, std::memory_order_relaxed);
			auto t = protocol::decode_motor_telemetry_2(frame);
			cached_m3_speed_ = t.motor_a.speed_mms;
			cached_m4_speed_ = t.motor_b.speed_mms;

			WheelSpeeds ws = {cached_m1_speed_, cached_m2_speed_,
			                  cached_m3_speed_, cached_m4_speed_};
			VehicleTwist vt = forward_kinematics(ws);
			odometry_integrate(odom_pose_, vt, 1.0 / 125.0);

			auto msg = nav_msgs::msg::Odometry();
			msg.header.stamp    = this->now();
			msg.header.frame_id = "odom";
			msg.child_frame_id  = "base_footprint";
			msg.pose.pose.position.x = odom_pose_.x;
			msg.pose.pose.position.y = odom_pose_.y;
			tf2::Quaternion q;
			q.setRPY(0, 0, odom_pose_.θ);
			msg.pose.pose.orientation.x = q.x();
			msg.pose.pose.orientation.y = q.y();
			msg.pose.pose.orientation.z = q.z();
			msg.pose.pose.orientation.w = q.w();
			msg.twist.twist.linear.x  = vt.v_x;
			msg.twist.twist.angular.z = vt.w_z;
			msg.pose.covariance[0]  = -1.0;
			msg.pose.covariance[7]  = -1.0;
			msg.pose.covariance[35] = -1.0;
			msg.twist.covariance[0] = -1.0;
			msg.twist.covariance[35]= -1.0;
			odom_pub_->publish(msg);
			break;
		}

		// ── 0x203: cache accel for next /imu ──────────────
		case 0x203: {
			frame_counts_[2].fetch_add(1, std::memory_order_relaxed);
			auto raw = protocol::decode_imu_accel(frame);
			auto si  = protocol::raw_to_si(raw);
			cached_accel_x_ = si.ax;
			cached_accel_y_ = si.ay;
			cached_accel_z_ = si.az;
			break;
		}

		// ── 0x204: gyro → publish /imu ───────────────────
		case 0x204: {
			frame_counts_[3].fetch_add(1, std::memory_order_relaxed);
			auto raw = protocol::decode_imu_gyro(frame);
			auto si  = protocol::raw_to_si(raw);

			auto msg = sensor_msgs::msg::Imu();
			msg.header.stamp    = this->now();
			msg.header.frame_id = "imu_link";

			msg.linear_acceleration.x = cached_accel_x_;
			msg.linear_acceleration.y = cached_accel_y_;
			msg.linear_acceleration.z = cached_accel_z_;

			msg.angular_velocity.x = si.gx * M_PI / 180.0;
			msg.angular_velocity.y = si.gy * M_PI / 180.0;
			msg.angular_velocity.z = si.gz * M_PI / 180.0;

			msg.linear_acceleration_covariance[0] = -1.0;
			msg.angular_velocity_covariance[0]    = -1.0;
			msg.orientation_covariance[0]          = -1.0;

			imu_pub_->publish(msg);
			break;
		}

		// ── 0x205: mag + temp (tracked, published via next /imu) ─
		case 0x205:
			frame_counts_[4].fetch_add(1, std::memory_order_relaxed);
			break;

		// ── 0x101: motor command (loopback disabled; STM32 may echo) ─
		case 0x101:
			frame_counts_[5].fetch_add(1, std::memory_order_relaxed);
			break;

		default:
			frame_counts_[6].fetch_add(1, std::memory_order_relaxed);
			break;
		}
	}

	RCLCPP_INFO(get_logger(), "CAN read thread exiting");
}

// ═══════════════════════════════════════════════════════════════
// CAN TX timer: 100Hz — send cached cmd_vel as 0x101
// ═══════════════════════════════════════════════════════════════

void CanGatewayNode::can_tx_timer_callback()
{
	int64_t now_ns = this->now().nanoseconds();
	int64_t last_ns = last_cmd_vel_ns_.load(std::memory_order_relaxed);

	double v = cached_vx_.load(std::memory_order_relaxed);
	double w = cached_wz_.load(std::memory_order_relaxed);

	// Timeout: >200ms without cmd_vel → zero speed (safety)
	if (now_ns - last_ns > 200'000'000LL) {
		v = 0.0;
		w = 0.0;
	}

	WheelSpeeds ws = inverse_kinematics(v, w);
	protocol::MotorCommand cmd = {ws.m1, ws.m2, ws.m3, ws.m4};

	struct can_frame frame = {};
	frame.can_id = 0x101;
	frame.len    = protocol::encode_motor_command(frame.data, cmd);

	if (!can_iface_.write(&frame)) {
		RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
			"CAN write error (0x101): %s", can_iface_.get_last_error().c_str());
	}
}

// ═══════════════════════════════════════════════════════════════
// Subscriptions
// ═══════════════════════════════════════════════════════════════

void CanGatewayNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	cached_vx_.store(msg->linear.x,  std::memory_order_relaxed);
	cached_wz_.store(msg->angular.z, std::memory_order_relaxed);
	last_cmd_vel_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);
}

void CanGatewayNode::estop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
	struct can_frame frame = {};
	frame.can_id = 0x102;
	frame.len    = 1;
	frame.data[0] = msg->data ? 0x01 : 0x00;

	if (!can_iface_.write(&frame)) {
		RCLCPP_ERROR(get_logger(), "CAN write error (0x102): %s",
			can_iface_.get_last_error().c_str());
	}
}

// ═══════════════════════════════════════════════════════════════
// Statistics
// ═══════════════════════════════════════════════════════════════

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
	if (!rclcpp::ok()) {
		rclcpp::shutdown();
		return 1;
	}

	node->start();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
