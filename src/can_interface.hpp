#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <optional>
#include <linux/can.h>

struct CANFrame {
    uint32_t id;
    std::vector<uint8_t> data;
};

class CANInterface {
public:
    CANInterface();
    ~CANInterface();

    // Open the CAN interface
    bool open(const std::string& interface_name);

    // Close the interface
    void close();

    // Send a CAN frame
    bool send(uint32_t id, const uint8_t* data, size_t len);
    bool send(uint32_t id, const std::vector<uint8_t>& data);

    // Receive a CAN frame with timeout (returns nullopt on timeout or error)
    std::optional<CANFrame> receive(int timeout_ms);

    // Check if interface is open
    bool isOpen() const { return socket_fd_ >= 0; }

    // Get the socket file descriptor (for external polling)
    int getFd() const { return socket_fd_; }

private:
    int socket_fd_;
    std::string interface_name_;
    int64_t last_error_time_;
    int error_count_;
};
