#include "robot_can_gateway/kinematics.hpp"
#include <cmath>
#include <algorithm>

namespace robot_can_gateway
{

VehicleTwist forward_kinematics(const WheelSpeeds &ws)
{
	// Average left and right side speeds (mm/s → m/s)
	double v_L = (ws.m1 + ws.m2) * 0.5 / 1000.0;
	double v_R = (ws.m3 + ws.m4) * 0.5 / 1000.0;

	// Wheel balance: compensate left/right effective radius asymmetry.
	// Positive balance → left boosted, right reduced.
	v_L *= (1.0 + WHEEL_BALANCE);
	v_R *= (1.0 - WHEEL_BALANCE);

	VehicleTwist t = {};
	t.v_x = (v_L + v_R) / 2.0;
	t.v_y = 0.0;
	t.w_z = (v_R - v_L) / WHEEL_BASE;
	return t;
}

WheelSpeeds inverse_kinematics(double v_x, double w_z)
{
	double v_L = v_x - w_z * WHEEL_BASE / 2.0;	// m/s
	double v_R = v_x + w_z * WHEEL_BASE / 2.0;

	// Pre-compensate left/right asymmetry so PID produces equal ground speed
	v_L *= (1.0 + WHEEL_BALANCE);
	v_R *= (1.0 - WHEEL_BALANCE);

	// Convert to mm/s, clamp to valid range, round to int16
	int16_t speed_L = (int16_t)std::clamp(v_L * 1000.0, -32768.0, 32767.0);
	int16_t speed_R = (int16_t)std::clamp(v_R * 1000.0, -32768.0, 32767.0);

	// Both wheels on each side get the same target
	WheelSpeeds ws = {};
	ws.m1 = speed_L;	// LB
	ws.m2 = speed_L;	// LF
	ws.m3 = speed_R;	// RF
	ws.m4 = speed_R;	// RB
	return ws;
}

void odometry_integrate(OdometryPose &pose, const VehicleTwist &twist, double dt)
{
	double dθ = twist.w_z * dt;
	pose.θ += dθ;

	double half_dθ = dθ / 2.0;
	pose.x += twist.v_x * std::cos(pose.θ - half_dθ) * dt;
	pose.y += twist.v_x * std::sin(pose.θ - half_dθ) * dt;

	// Normalize yaw to [-π, π]
	pose.θ = std::fmod(pose.θ + M_PI, 2.0 * M_PI);
	if (pose.θ < 0) pose.θ += 2.0 * M_PI;
	pose.θ -= M_PI;
}

} // namespace robot_can_gateway
