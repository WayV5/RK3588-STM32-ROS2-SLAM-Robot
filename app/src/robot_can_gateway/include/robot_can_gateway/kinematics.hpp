#pragma once

#include <cstdint>

namespace robot_can_gateway
{

// ---- Robot physical parameters ----

constexpr double WHEEL_BASE     = 0.275;	// 左右轮中心距, 卷尺精准测量 275mm (2026-06-14)
constexpr double WHEEL_RADIUS   = 0.0323;	// effective radius (m), 标称65mm→实测有效~64.6mm (2026-06-14 二次标定, WHEEL_BALANCE=0.003)
constexpr double GEAR_RATIO     = 30.0;	// motor gear reduction
constexpr double ODOMETRY_DT    = 1.0 / 125.0;	// odom integration period (s), matches STM32 0x201 @ 125Hz

// Wheel balance: compensates left/right effective radius asymmetry.
// Positive → left side boosted, right side reduced. Calibrate until angular.z≈0.
// v_L *= (1+balance), v_R *= (1-balance)
constexpr double WHEEL_BALANCE  = 0.003;	// calibrated: angular.z=0.0008 @ 0.38m/s (2026-06-14)

// ---- Forward kinematics: 4 wheel speeds (mm/s) → Twist (m/s, rad/s) ----

struct WheelSpeeds {
	int16_t m1;	// M1 LB left-back
	int16_t m2;	// M2 LF left-front
	int16_t m3;	// M3 RF right-front
	int16_t m4;	// M4 RB right-back
};

struct VehicleTwist {
	double v_x;	// linear velocity (m/s)
	double v_y;	// lateral velocity — 0 for differential drive
	double w_z;	// angular velocity (rad/s)
};

// Average left side and right side, then forward kinematics.
VehicleTwist forward_kinematics(const WheelSpeeds &ws);

// ---- Inverse kinematics: Twist (m/s, rad/s) → 4 wheel speeds (mm/s) ----

// Converts chassis Twist to 4 wheel targets in mm/s.
// Left side wheels share v_L, right side share v_R.
WheelSpeeds inverse_kinematics(double v_x, double w_z);

// ---- Odometry integration (Euler method) ----

struct OdometryPose {
	double x;	// world X (m)
	double y;	// world Y (m)
	double θ;	// yaw (rad)
};

// Integrate one step. dt should be ODOMETRY_DT (1/125s = 0.008s), matching STM32 0x201 rate.
void odometry_integrate(OdometryPose &pose, const VehicleTwist &twist, double dt);

} // namespace robot_can_gateway
