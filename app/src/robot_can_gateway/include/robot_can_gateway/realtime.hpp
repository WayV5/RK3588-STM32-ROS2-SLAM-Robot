#pragma once

#include <cerrno>
#include <cstring>
#include <sched.h>
#include <sys/mman.h>

namespace robot_can_gateway
{

/*
 * Set SCHED_FIFO real-time scheduling policy and lock memory.
 *
 * Requires CAP_SYS_NICE.  Before running on the target board:
 *   sudo setcap cap_sys_nice=ep <install_prefix>/lib/robot_can_gateway/can_gateway_node
 *
 * Returns 0 on success, or -1 with errno set on failure.
 * Memory locking (mlockall) is best-effort — failure is non-fatal.
 */
inline int set_realtime_priority(int priority)
{
	struct sched_param param;
	param.sched_priority = priority;

	if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
		return -1;
	}

	(void)mlockall(MCL_CURRENT | MCL_FUTURE);

	return 0;
}

} // namespace robot_can_gateway
