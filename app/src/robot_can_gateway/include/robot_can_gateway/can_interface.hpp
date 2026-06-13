#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>

namespace robot_can_gateway
{

class CanInterface
{
public:
	CanInterface();
	~CanInterface();

	// Non-copyable (owns a file descriptor)
	CanInterface(const CanInterface &) = delete;
	CanInterface &operator=(const CanInterface &) = delete;

	// Open a socketCAN interface (e.g. "can0").
	// Returns true on success. On failure, call get_last_error() for details.
	bool open(const std::string &ifname);

	// Blocking read of one CAN frame.
	// Returns true iff a complete can_frame was read.
	bool read(struct can_frame *frame);

	// Write one CAN frame.
	// Returns true iff the full frame was written to the socket.
	bool write(const struct can_frame *frame);

	// Close the socket. Safe to call multiple times.
	void close();

	// True between a successful open() and close().
	bool is_open() const;

	// Human-readable description of the last error.
	const std::string &get_last_error() const;

private:
	int sockfd_;
	bool is_open_;
	std::string last_error_;
};

} // namespace robot_can_gateway
