#include "motor_protocol.hpp"

#include <chrono>
#include <cstring>
#include <iostream>

MotorProtocol::MotorProtocol(CANInterface& can) : can_(can) {}

bool MotorProtocol::setMode(uint8_t mode) {
    uint8_t data[2] = {mode, DEVICE_ID};
    bool success = can_.send(CAN_ID_NMT, data, 2);
    if (success) {
        state_.mode = mode;
        std::cout << "Set motor mode to 0x" << std::hex << (int)mode << std::dec << std::endl;
    }
    return success;
}

bool MotorProtocol::setPositionMode() {
    return setMode(MODE_POSITION);
}

bool MotorProtocol::sendPositionCommand(float position_rad, float velocity_rad_s) {
    uint8_t data[8];
    encodeFloat(position_rad, data);
    encodeFloat(velocity_rad_s, data + 4);
    return can_.send(CAN_ID_PDO2_TX, data, 8);
}

bool MotorProtocol::sendHeartbeat() {
    // Send empty heartbeat frame
    return can_.send(CAN_ID_HEARTBEAT, nullptr, 0);
}

bool MotorProtocol::parseFrame(const CANFrame& frame) {
    bool updated = false;

    if (frame.id == CAN_ID_PDO2_RX || frame.id == CAN_ID_FAST_FRAME) {
        if (frame.data.size() >= 8) {
            state_.position_rad = decodeFloat(frame.data.data());
            state_.velocity_rad_s = decodeFloat(frame.data.data() + 4);
            state_.last_feedback_time_ms = getCurrentTimeMs();
            state_.connected = true;
            updated = true;
        }
    } else if (frame.id == CAN_ID_HEARTBEAT) {
        // Motor is alive on the bus — keep connection alive even without position feedback
        state_.last_feedback_time_ms = getCurrentTimeMs();
        state_.connected = true;
        updated = true;
    }

    return updated;
}

bool MotorProtocol::isConnected(uint64_t timeout_ms) const {
    if (!state_.connected) return false;
    uint64_t elapsed = getCurrentTimeMs() - state_.last_feedback_time_ms;
    return elapsed < timeout_ms;
}

uint64_t MotorProtocol::getCurrentTimeMs() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

void MotorProtocol::encodeFloat(float value, uint8_t* buffer) {
    std::memcpy(buffer, &value, sizeof(float));
}

float MotorProtocol::decodeFloat(const uint8_t* buffer) {
    float value;
    std::memcpy(&value, buffer, sizeof(float));
    return value;
}
