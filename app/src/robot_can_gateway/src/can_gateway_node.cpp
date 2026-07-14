#include "robot_can_gateway/can_gateway_node.hpp"
#include "robot_can_gateway/can_protocol.hpp"
#include "robot_can_gateway/realtime.hpp"

#include <chrono>
#include <cmath>

namespace robot_can_gateway
{

CanGatewayNode::CanGatewayNode(const std::string &can_iface)
	: Node("can_gateway")
	, odom_pose_{0.0, 0.0, 0.0}
	, cached_accel_x_(0.0)
	, cached_accel_y_(0.0)
	, cached_accel_z_(0.0)
	, cached_mag_x_(0.0)
	, cached_mag_y_(0.0)
	, cached_mag_z_(0.0)
{
	this->declare_parameter("can_interface", can_iface);

	std::string ifname = this->get_parameter("can_interface").as_string();

	if (!can_iface_.open(ifname)) {
		RCLCPP_ERROR(get_logger(), "Failed to open CAN interface '%s': %s",
			ifname.c_str(), can_iface_.get_last_error().c_str());
		// Don't shutdown — component may be loaded in a shared container.
		// Node stays alive but all telemetry processing will be no-ops.
		return;
	}

	RCLCPP_INFO(get_logger(), "Opened CAN interface '%s'", ifname.c_str());

	for (auto &c : frame_counts_) {
		c.store(0, std::memory_order_relaxed);
	}

	cached_vx_.store(0.0, std::memory_order_relaxed);
	cached_wz_.store(0.0, std::memory_order_relaxed);
	last_cmd_vel_ns_.store(0, std::memory_order_relaxed);
	last_heartbeat_ns_.store(0, std::memory_order_relaxed);
	last_can_rx_ns_.store(0, std::memory_order_relaxed);
	memset(prev_counts_, 0, sizeof(prev_counts_));

	// Publishers
	odom_raw_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_raw",
		rclcpp::QoS(1).best_effort());
	imu_pub_  = this->create_publisher<sensor_msgs::msg::Imu>("/imu",
		rclcpp::QoS(1).best_effort());
	mag_pub_  = this->create_publisher<sensor_msgs::msg::MagneticField>("/mag",
		rclcpp::QoS(1).best_effort());
	diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
		"/diagnostics", rclcpp::QoS(1).reliable());

	// Subscriptions
	cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
		"/cmd_vel", rclcpp::QoS(10).reliable(),
		std::bind(&CanGatewayNode::cmd_vel_callback, this, std::placeholders::_1));

	estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
		"/emergency_stop", rclcpp::QoS(1).reliable(),
		std::bind(&CanGatewayNode::estop_callback, this, std::placeholders::_1));

	// Timers
	process_timer_ = this->create_wall_timer(std::chrono::milliseconds(4),
		std::bind(&CanGatewayNode::process_telemetry, this));

	can_tx_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
		std::bind(&CanGatewayNode::can_tx_timer_callback, this));

	stats_timer_ = this->create_wall_timer(std::chrono::seconds(1),
		std::bind(&CanGatewayNode::publish_diagnostics, this));

}


CanGatewayNode::~CanGatewayNode()
{
	running_.store(false, std::memory_order_relaxed);
	if (can_thread_.joinable()) {
		can_thread_.join();
	}
	can_iface_.close();
}

// ═══════════════════════════════════════════════════════════════
// Thread 1: CAN read — only push to ring buffer
// ═══════════════════════════════════════════════════════════════

void CanGatewayNode::can_read_loop()
{
	// SCHED_FIFO 80 — highest priority for CAN I/O thread
	if (set_realtime_priority(80) != 0) {
		RCLCPP_WARN(get_logger(),
			"Failed to set SCHED_FIFO 80 for CAN read thread: %s "
			"(try: sudo setcap cap_sys_nice=ep <binary>)", strerror(errno));
	}

	struct can_frame frame;

	while (running_.load(std::memory_order_relaxed)) {
		if (!can_iface_.read(&frame)) {
			RCLCPP_ERROR(get_logger(), "CAN read error: %s",
				can_iface_.get_last_error().c_str());
			break;
		}

		if (!ringbuf_.push(frame)) {
			// Ring buffer full — frame dropped
			frame_counts_[6].fetch_add(1, std::memory_order_relaxed);	// overflow count
		}
	}

	RCLCPP_INFO(get_logger(), "CAN read thread exiting");
}

// ═══════════════════════════════════════════════════════════════
// Thread 2: 250Hz timer — drain ring buffer, decode, publish
// ═══════════════════════════════════════════════════════════════

void CanGatewayNode::process_telemetry()
{
	struct can_frame frames[64];
	size_t n = ringbuf_.pop_all(frames, 64);

	for (size_t i = 0; i < n; i++) {
		const auto &f = frames[i];
		uint32_t id = f.can_id & CAN_SFF_MASK;
		frame_counts_[7].fetch_add(1, std::memory_order_relaxed);
		last_can_rx_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);

		switch (id) {

		// ── 0x201: 4-wheel speeds → /odom_raw (125Hz, one frame = all 4 wheels) ─
		case 0x201: {
			frame_counts_[0].fetch_add(1, std::memory_order_relaxed);
			auto s = protocol::decode_motor_speeds(f);
			WheelSpeeds ws = {s.m1, s.m2, s.m3, s.m4};
			VehicleTwist vt = forward_kinematics(ws);
			odometry_integrate(odom_pose_, vt, ODOMETRY_DT);

			auto msg = nav_msgs::msg::Odometry();
			msg.header.stamp    = this->now();
			msg.header.frame_id = "odom";
			msg.child_frame_id  = "base_footprint";
			msg.pose.pose.position.x = odom_pose_.x;
			msg.pose.pose.position.y = odom_pose_.y;
			double half_θ = odom_pose_.θ * 0.5;
			msg.pose.pose.orientation.z = std::sin(half_θ);
			msg.pose.pose.orientation.w = std::cos(half_θ);
			msg.twist.twist.linear.x  = vt.v_x;
			msg.twist.twist.angular.z = vt.w_z;
			msg.pose.covariance[0]  = -1.0;
			msg.pose.covariance[7]  = -1.0;
			msg.pose.covariance[35] = -1.0;
			msg.twist.covariance[0] = 0.0226;	// calibrated 0.3m/s straight
			msg.twist.covariance[35]= 0.0613;	// calibrated 0.5rad/s rotation
			publish_odom_raw(msg);

			// TF odom→base_footprint now published by EKF (robot_localization)
			break;
		}

		// ── 0x202: 4-wheel PWM — count only (diagnostics) ─
		case 0x202:
			frame_counts_[1].fetch_add(1, std::memory_order_relaxed);
			break;

		// ── 0x203: cache accel ────────────────────────────
		case 0x203: {
			frame_counts_[2].fetch_add(1, std::memory_order_relaxed);
			auto raw = protocol::decode_imu_accel(f);
			auto si  = protocol::raw_to_si(raw);
			cached_accel_x_ = si.ax;
			cached_accel_y_ = si.ay;
			cached_accel_z_ = si.az;
			break;
		}

		// ── 0x204: gyro + cached accel → /imu ─────────────
		case 0x204: {
			frame_counts_[3].fetch_add(1, std::memory_order_relaxed);
			auto raw = protocol::decode_imu_gyro(f);
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
			publish_imu(msg);
			break;
		}

		// ── 0x205: mag+temp → /mag (20Hz) ────────────────
		case 0x205: {
			frame_counts_[4].fetch_add(1, std::memory_order_relaxed);
			auto raw = protocol::decode_imu_mag_temp(f);
			auto si  = protocol::raw_to_si(raw);
			cached_mag_x_ = si.mx;
			cached_mag_y_ = si.my;
			cached_mag_z_ = si.mz;

			auto msg = sensor_msgs::msg::MagneticField();
			msg.header.stamp    = this->now();
			msg.header.frame_id = "imu_link";
			msg.magnetic_field.x = si.mx * 1e-6;	// µT → T
			msg.magnetic_field.y = si.my * 1e-6;
			msg.magnetic_field.z = si.mz * 1e-6;
			msg.magnetic_field_covariance[0] = -1.0;
			mag_pub_->publish(msg);
			break;
		}

		// ── 0x206: system status — count + heartbeat ──────
		case 0x206: {
			frame_counts_[5].fetch_add(1, std::memory_order_relaxed);
			last_heartbeat_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);
			// Log status flags on change
			static uint8_t last_flags = 0xFF;
			auto st = protocol::decode_sys_status(f);
			if (st.flags != last_flags) {
				last_flags = st.flags;
				RCLCPP_INFO(get_logger(), "System status: flags=0x%02X fault=0x%02X",
					st.flags, st.fault_code);
			}
			break;
		}

		// ── 0x101: motor command echo (loopback disabled, STM32 may echo) ─
		case 0x101:
			// counted separately — not telemetry, counted in frame_counts_[6] below
			break;

		default:
			break;
		}
	}
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
// Loan API helpers — zero-copy publish (falls back to regular publish)
// ═══════════════════════════════════════════════════════════════

void CanGatewayNode::publish_odom_raw(const nav_msgs::msg::Odometry &msg)
{
	auto loaned = odom_raw_pub_->borrow_loaned_message();
	if (loaned.is_valid()) {
		loaned.get() = msg;
		odom_raw_pub_->publish(std::move(loaned));
	} else {
		odom_raw_pub_->publish(msg);
	}
}

void CanGatewayNode::publish_imu(const sensor_msgs::msg::Imu &msg)
{
	auto loaned = imu_pub_->borrow_loaned_message();
	if (loaned.is_valid()) {
		loaned.get() = msg;
		imu_pub_->publish(std::move(loaned));
	} else {
		imu_pub_->publish(msg);
	}
}

// ═══════════════════════════════════════════════════════════════
// Diagnostics (1Hz)
// ═══════════════════════════════════════════════════════════════

void CanGatewayNode::publish_diagnostics()
{
	auto total = frame_counts_[7].load(std::memory_order_relaxed);
	auto drop  = frame_counts_[6].load(std::memory_order_relaxed);

	// Per-second rates
	uint64_t rates[8];
	for (int i = 0; i < 8; i++) {
		uint64_t cur = (i == 7) ? total : ((i == 6) ? drop :
			frame_counts_[i].load(std::memory_order_relaxed));
		rates[i] = cur - prev_counts_[i];
		prev_counts_[i] = cur;
	}

	int64_t now_ns = this->now().nanoseconds();
	int64_t hb_ns  = last_heartbeat_ns_.load(std::memory_order_relaxed);
	int64_t rx_ns  = last_can_rx_ns_.load(std::memory_order_relaxed);
	double hb_age  = (hb_ns > 0) ? (now_ns - hb_ns) / 1e9 : -1.0;
	double rx_age  = (rx_ns > 0) ? (now_ns - rx_ns) / 1e9 : -1.0;

	diagnostic_msgs::msg::DiagnosticArray diag;
	diag.header.stamp = this->now();

	auto make_status = [&](const char *name, int idx, double expected_hz,
	                        double lo_hz, const char *desc) {
		diagnostic_msgs::msg::DiagnosticStatus s;
		s.name = name;
		bool ok = (rates[idx] >= lo_hz);
		s.level = ok ? diagnostic_msgs::msg::DiagnosticStatus::OK
		             : diagnostic_msgs::msg::DiagnosticStatus::WARN;
		s.message = ok
			? std::string(desc) + ": " + std::to_string(rates[idx]) + " Hz (OK)"
			: std::string(desc) + ": " + std::to_string(rates[idx])
				+ " Hz (expected ~" + std::to_string((int)expected_hz) + " Hz)";
		return s;
	};

	// ── Telemetry frame rates ──────────────────────────────
	diag.status.push_back(make_status("CAN 0x201 Speeds", 0, 125, 50, "Motor speeds"));
	diag.status.push_back(make_status("CAN 0x202 PWM",    1,  10,  2, "Motor PWM"));
	diag.status.push_back(make_status("CAN 0x203 Accel",  2, 250, 50, "Accelerometer"));
	diag.status.push_back(make_status("CAN 0x204 Gyro",   3, 250, 50, "Gyroscope"));
	diag.status.push_back(make_status("CAN 0x205 Mag",    4,  20,  5, "Magnetometer"));

	// ── STM32 heartbeat (0x206 @ 1Hz) ─────────────────────
	{
		diagnostic_msgs::msg::DiagnosticStatus s;
		s.name = "STM32 Heartbeat";
		s.values.push_back(
			diagnostic_msgs::msg::KeyValue()
			.set__key("0x206_rate").set__value(std::to_string(rates[5]) + " Hz"));
		s.values.push_back(
			diagnostic_msgs::msg::KeyValue()
			.set__key("last_0x206_age").set__value(std::to_string(hb_age) + " s"));

		if (hb_age < 0) {
			s.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
			s.message = "No heartbeat received yet";
		} else if (hb_age > 5.0) {
			s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
			s.message = "STM32 heartbeat lost (>5s)";
		} else if (hb_age > 2.0) {
			s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
			s.message = "STM32 heartbeat late (>2s)";
		} else {
			s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
			s.message = "Heartbeat OK (" + std::to_string((int)(hb_age * 1000)) + "ms ago)";
		}
		diag.status.push_back(s);
	}

	// ── Ring buffer ───────────────────────────────────────
	{
		diagnostic_msgs::msg::DiagnosticStatus s;
		s.name = "Ring Buffer";
		s.values.push_back(
			diagnostic_msgs::msg::KeyValue()
			.set__key("drops_per_sec").set__value(std::to_string(rates[6])));
		if (rates[6] > 0) {
			s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
			s.message = std::to_string(rates[6]) + " frames dropped this second";
		} else {
			s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
			s.message = "No drops";
		}
		diag.status.push_back(s);
	}

	// ── CAN bus liveness ──────────────────────────────────
	{
		diagnostic_msgs::msg::DiagnosticStatus s;
		s.name = "CAN Bus";
		s.values.push_back(
			diagnostic_msgs::msg::KeyValue()
			.set__key("total_fps").set__value(std::to_string(rates[7]) + " Hz"));
		s.values.push_back(
			diagnostic_msgs::msg::KeyValue()
			.set__key("last_rx_age").set__value(std::to_string(rx_age) + " s"));

		if (rx_age < 0) {
			s.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
			s.message = "No CAN data received yet";
		} else if (rx_age > 1.0) {
			s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
			s.message = "CAN bus silent (>1s) — check wiring/STM32";
		} else if (rx_age > 0.5) {
			s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
			s.message = "CAN bus gaps detected";
		} else if (rates[7] < 300) {
			s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
			s.message = "Total fps low: " + std::to_string(rates[7]) + " (expected ~700)";
		} else {
			s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
			s.message = "CAN bus active (" + std::to_string(rates[7]) + " fps)";
		}
		diag.status.push_back(s);
	}

	diag_pub_->publish(diag);

	// Also keep the log line for quick terminal view
	RCLCPP_INFO(get_logger(),
		"diag: Speeds=%luHz PWM=%luHz Accel=%luHz Gyro=%luHz Mag=%luHz "
		"HBeat=%luHz total=%luHz drops=%lu hb_age=%.1fs",
		rates[0], rates[1], rates[2], rates[3], rates[4],
		rates[5], rates[7], rates[6], hb_age);
}

} // namespace robot_can_gateway

void robot_can_gateway::CanGatewayNode::start()
{
	running_.store(true, std::memory_order_relaxed);
	can_thread_ = std::thread(&robot_can_gateway::CanGatewayNode::can_read_loop, this);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<robot_can_gateway::CanGatewayNode>();
	if (!rclcpp::ok()) {
		rclcpp::shutdown();
		return 1;
	}

	node->start();

	// SCHED_FIFO 75 — executor thread (timers + callbacks + publish)
	if (robot_can_gateway::set_realtime_priority(75) != 0) {
		RCLCPP_WARN(rclcpp::get_logger("can_gateway"),
			"Failed to set SCHED_FIFO 75 for executor thread: %s", strerror(errno));
	}

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
