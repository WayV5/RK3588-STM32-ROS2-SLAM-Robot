#include "robot_can_gateway/can_interface.hpp"

#include <cstring>
#include <cerrno>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>

namespace robot_can_gateway
{

CanInterface::CanInterface()
	: sockfd_(-1)
	, is_open_(false)
{
}

CanInterface::~CanInterface()
{
	close();
}

bool CanInterface::open(const std::string &ifname)
{
	if (is_open_) {
		close();
	}

	sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (sockfd_ < 0) {
		last_error_ = std::string("socket() failed: ") + strerror(errno);
		return false;
	}

	struct ifreq ifr = {};
	strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);

	if (ioctl(sockfd_, SIOCGIFINDEX, &ifr) < 0) {
		last_error_ = std::string("ioctl(SIOCGIFINDEX) for ") + ifname
			+ " failed: " + strerror(errno);
		::close(sockfd_);
		sockfd_ = -1;
		return false;
	}

	struct sockaddr_can addr = {};
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(sockfd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		last_error_ = std::string("bind() failed: ") + strerror(errno);
		::close(sockfd_);
		sockfd_ = -1;
		return false;
	}

	// Disable local loopback: we don't need to see our own transmitted frames
	int loopback = 0;
	setsockopt(sockfd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &loopback, sizeof(loopback));

	is_open_ = true;
	last_error_.clear();
	return true;
}

bool CanInterface::read(struct can_frame *frame)
{
	if (!is_open_) {
		last_error_ = "socket not open";
		return false;
	}

	ssize_t n = ::read(sockfd_, frame, sizeof(struct can_frame));
	if (n < 0) {
		last_error_ = std::string("read() failed: ") + strerror(errno);
		return false;
	}
	if (n != (ssize_t)sizeof(struct can_frame)) {
		last_error_ = "read() returned incomplete frame";
		return false;
	}
	return true;
}

bool CanInterface::write(const struct can_frame *frame)
{
	if (!is_open_) {
		last_error_ = "socket not open";
		return false;
	}

	ssize_t n = ::write(sockfd_, frame, sizeof(struct can_frame));
	if (n < 0) {
		last_error_ = std::string("write() failed: ") + strerror(errno);
		return false;
	}
	if (n != (ssize_t)sizeof(struct can_frame)) {
		last_error_ = "write() incomplete frame";
		return false;
	}
	return true;
}

void CanInterface::close()
{
	if (sockfd_ >= 0) {
		::close(sockfd_);
		sockfd_ = -1;
	}
	is_open_ = false;
}

bool CanInterface::is_open() const
{
	return is_open_;
}

const std::string &CanInterface::get_last_error() const
{
	return last_error_;
}

} // namespace robot_can_gateway
