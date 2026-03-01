#include "can_interface.hpp"

#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>
#include <iostream>
#include <chrono>

CANInterface::CANInterface() : socket_fd_(-1), last_error_time_(0), error_count_(0) {}

CANInterface::~CANInterface() {
    close();
}

bool CANInterface::open(const std::string& interface_name) {
    interface_name_ = interface_name;

    // Create socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cerr << "Error creating CAN socket: " << strerror(errno) << std::endl;
        return false;
    }

    // Get interface index
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Error getting interface index for " << interface_name
                  << ": " << strerror(errno) << std::endl;
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Enable receiving own transmitted frames (echo) so we see our heartbeats
    int recv_own = 1;
    setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own));

    // Increase receive buffer
    int rcvbuf = 65536;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    // Bind socket to CAN interface
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error binding CAN socket: " << strerror(errno) << std::endl;
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    std::cout << "CAN interface " << interface_name << " opened successfully" << std::endl;
    return true;
}

void CANInterface::close() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool CANInterface::send(uint32_t id, const uint8_t* data, size_t len) {
    if (socket_fd_ < 0) {
        return false;
    }

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = id;
    frame.can_dlc = std::min(len, static_cast<size_t>(8));
    std::memcpy(frame.data, data, frame.can_dlc);

    ssize_t nbytes = write(socket_fd_, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        error_count_++;
        auto now = std::chrono::steady_clock::now().time_since_epoch();
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();

        // Only log errors every 5 seconds to avoid spam
        if (now_ms - last_error_time_ > 5000) {
            std::cerr << "CAN send error: " << strerror(errno);
            if (error_count_ > 1) {
                std::cerr << " (" << error_count_ << " errors)";
            }
            std::cerr << " - check motor connection" << std::endl;
            last_error_time_ = now_ms;
            error_count_ = 0;
        }
        return false;
    }

    return true;
}

bool CANInterface::send(uint32_t id, const std::vector<uint8_t>& data) {
    return send(id, data.data(), data.size());
}

std::optional<CANFrame> CANInterface::receive(int timeout_ms) {
    if (socket_fd_ < 0) {
        return std::nullopt;
    }

    // Use poll for timeout
    struct pollfd pfd;
    pfd.fd = socket_fd_;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, timeout_ms);
    if (ret < 0) {
        std::cerr << "Poll error: " << strerror(errno) << std::endl;
        return std::nullopt;
    }
    if (ret == 0) {
        // Timeout
        return std::nullopt;
    }

    // Read frame
    struct can_frame frame;
    ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        return std::nullopt;
    }

    CANFrame result;
    result.id = frame.can_id;
    result.data.assign(frame.data, frame.data + frame.can_dlc);

    return result;
}
